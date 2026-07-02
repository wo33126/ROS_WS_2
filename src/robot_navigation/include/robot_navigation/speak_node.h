#pragma once

#include <ros/ros.h>
#include <robot_navigation/Speak.h>

#include <string>

namespace robot_navigation {

/**
 * @brief 语音播报节点
 *
 * 提供 /speak 服务，将文本转换为语音输出。
 * 优先使用 espeak-ng 文字转语音引擎，回退到 ROS 日志输出。
 *
 * 参数:
 *   language: 语音语言 (默认 "zh")
 *   speed:    语速倍率 (默认 1.0)
 *   volume:   音量 (默认 1.0)
 *   use_espeak: 是否使用 espeak-ng (默认 true)
 */
class SpeakNode {
 public:
  SpeakNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~SpeakNode() = default;

  bool init();

 private:
  bool handleSpeak(robot_navigation::Speak::Request& req,
                   robot_navigation::Speak::Response& res);

  bool speakEspeak(const std::string& text);
  bool speakSystem(const std::string& text);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::ServiceServer speak_srv_;

  // 参数
  std::string language_;
  double speed_;
  double volume_;
  bool use_espeak_;
  std::string espeak_voice_;
  std::string fallback_cmd_;
};

}  // namespace robot_navigation
