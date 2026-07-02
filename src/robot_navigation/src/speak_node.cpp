/**
 * @file speak_node.cpp
 * @brief 语音播报服务节点实现
 *
 * 提供 /speak 服务，将文本转换为语音输出。
 * 支持 espeak-ng（TTS 引擎）和系统命令回退。
 */

#include "robot_navigation/speak_node.h"

#include <cstdlib>
#include <cstdio>
#include <array>
#include <sstream>
#include <thread>

namespace robot_navigation {

// ── 构造 ───────────────────────────────────────────────────
SpeakNode::SpeakNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , language_("zh")
    , speed_(1.0)
    , volume_(1.0)
    , use_espeak_(true)
    , espeak_voice_("zh")
    , fallback_cmd_("")
{
  pnh_.param<std::string>("language", language_, "zh");
  pnh_.param<double>("speed", speed_, 1.0);
  pnh_.param<double>("volume", volume_, 1.0);
  pnh_.param<bool>("use_espeak", use_espeak_, true);
  pnh_.param<std::string>("espeak_voice", espeak_voice_, "zh");
  pnh_.param<std::string>("fallback_cmd", fallback_cmd_, "");
}

// ── 初始化 ─────────────────────────────────────────────────
bool SpeakNode::init() {
  speak_srv_ = nh_.advertiseService("/speak",
                                    &SpeakNode::handleSpeak, this);

  // 检测 espeak-ng 是否可用
  if (use_espeak_) {
    int ret = std::system("which espeak-ng > /dev/null 2>&1");
    if (ret != 0) {
      ret = std::system("which espeak > /dev/null 2>&1");
      if (ret != 0) {
        ROS_WARN("[speak_node] espeak-ng/espeak 未安装，将使用 ROS 日志输出代替");
        use_espeak_ = false;
      }
    }
  }

  // 检测其他 TTS 引擎
  if (!use_espeak_ && fallback_cmd_.empty()) {
    // 尝试 festival
    int ret = std::system("which festival > /dev/null 2>&1");
    if (ret == 0) {
      fallback_cmd_ = "echo \"%s\" | festival --tts";
    }
  }

  ROS_INFO("[speak_node] 初始化完成");
  ROS_INFO("[speak_node]   语言: %s, 语速: %.1fx, 音量: %.1f", language_.c_str(), speed_, volume_);
  if (use_espeak_) {
    ROS_INFO("[speak_node]   引擎: espeak-ng (voice=%s)", espeak_voice_.c_str());
  } else if (!fallback_cmd_.empty()) {
    ROS_INFO("[speak_node]   引擎: festival 回退");
  } else {
    ROS_INFO("[speak_node]   引擎: ROS 日志输出（无 TTS 引擎可用）");
  }
  ROS_INFO("[speak_node]   /speak 服务已就绪");

  return true;
}

// ── /speak 服务回调 ─────────────────────────────────────────
bool SpeakNode::handleSpeak(robot_navigation::Speak::Request& req,
                            robot_navigation::Speak::Response& res) {
  if (req.text.empty()) {
    res.success = false;
    res.message = "播报文本为空";
    ROS_WARN("[speak_node] %s", res.message.c_str());
    return true;
  }

  ROS_INFO("[speak_node] 🎤 播报: \"%s\"", req.text.c_str());

  bool ok = false;

  // 优先使用 espeak-ng
  if (use_espeak_) {
    ok = speakEspeak(req.text);
  }

  // 回退到系统命令
  if (!ok && !fallback_cmd_.empty()) {
    ok = speakSystem(req.text);
  }

  if (ok) {
    res.success = true;
    res.message = "语音播报完成";
  } else {
    // 最终回退：ROS 日志输出（保证服务不会失败）
    res.success = true;  // 不阻止任务流程
    res.message = "语音播报已通过 ROS 日志输出（TTS 引擎不可用）";
    ROS_WARN("[speak_node] TTS 引擎不可用，文本已通过日志输出");
  }

  return true;
}

// ── espeak-ng 文字转语音 ─────────────────────────────────────
bool SpeakNode::speakEspeak(const std::string& text) {
  // 转义特殊字符（双引号和反斜杠）
  std::string escaped;
  escaped.reserve(text.size() * 2);
  for (char c : text) {
    if (c == '"' || c == '\\') {
      escaped += '\\';
    }
    escaped += c;
  }

  // 构建 espeak-ng 命令
  std::ostringstream cmd;
  cmd << "espeak-ng -v " << espeak_voice_
      << " -s " << static_cast<int>(175.0 * speed_)
      << " -a " << static_cast<int>(100.0 * volume_)
      << " \"" << escaped << "\""
      << " 2>/dev/null";

  int ret = std::system(cmd.str().c_str());
  if (ret != 0) {
    // 尝试用 espeak 代替
    std::ostringstream cmd2;
    cmd2 << "espeak -v " << espeak_voice_
         << " -s " << static_cast<int>(175.0 * speed_)
         << " \"" << escaped << "\""
         << " 2>/dev/null";
    ret = std::system(cmd2.str().c_str());
  }

  return (ret == 0);
}

// ── 系统命令文字转语音（festival 等） ────────────────────────
bool SpeakNode::speakSystem(const std::string& text) {
  if (fallback_cmd_.empty()) return false;

  std::string escaped;
  escaped.reserve(text.size() * 2);
  for (char c : text) {
    if (c == '"' || c == '\\') {
      escaped += '\\';
    }
    escaped += c;
  }

  // 用文本替换命令中的 %s
  std::string cmd = fallback_cmd_;
  size_t pos = cmd.find("%s");
  if (pos != std::string::npos) {
    cmd.replace(pos, 2, escaped);
  } else {
    cmd += " \"" + escaped + "\"";
  }

  cmd += " 2>/dev/null";
  int ret = std::system(cmd.c_str());
  return (ret == 0);
}

}  // namespace robot_navigation

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "speak_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::SpeakNode node(nh, pnh);
  if (!node.init()) {
    ROS_FATAL("[speak_node] 初始化失败");
    return 1;
  }

  ros::spin();
  return 0;
}
