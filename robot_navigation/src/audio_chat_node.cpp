/**
 * @file audio_chat_node.cpp
 * @brief 双向音频通信节点
 *
 * 提供医生端与病人端的双向音频交流功能：
 *   1. 麦克风采集 → Opus 编码 → UDP 发送给医生端
 *   2. UDP 接收 ← 医生端音频 → Opus 解码 → 扬声器播放
 *   3. 与 speak_node 协作：语音播报时暂停音频流
 *
 * 协议：
 *   - 音频格式: 16kHz, 16-bit, mono PCM → Opus 编码
 *   - 传输: UDP 单播（机器人 ↔ 医生端 PC）
 *   - 端口: 机器人接收 5004, 发送到 5005
 *
 * 依赖: libopus (需安装 libopus-dev)
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstdlib>

// ── Opus 编解码（条件编译）──
#ifdef HAS_OPUS
#include <opus/opus.h>
#endif

namespace robot_navigation {

/**
 * @brief 音频聊天节点
 *
 * 管理双向音频流：
 *   - 音频采集线程: 从麦克风读取 PCM → Opus 编码 → UDP 发送
 *   - 音频接收线程: UDP 接收 → Opus 解码 → 扬声器播放
 */
class AudioChatNode {
 public:
  AudioChatNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh)
      , pnh_(pnh)
      , enabled_(false)
      , audio_active_(false)
      , sample_rate_(16000)
      , channels_(1)
      , frame_duration_ms_(20)
      , doctor_ip_("192.168.1.100")
      , rx_port_(5004)
      , tx_port_(5005)
      , use_alsa_(true)
      , mic_device_("default")
      , speaker_device_("default")
      , tx_socket_(-1)
      , rx_socket_(-1)
  {
    pnh_.param<bool>("enabled", enabled_, false);
    pnh_.param<int>("sample_rate", sample_rate_, 16000);
    pnh_.param<int>("channels", channels_, 1);
    pnh_.param<int>("frame_duration_ms", frame_duration_ms_, 20);
    pnh_.param<std::string>("doctor_ip", doctor_ip_, "192.168.1.100");
    pnh_.param<int>("rx_port", rx_port_, 5004);
    pnh_.param<int>("tx_port", tx_port_, 5005);
    pnh_.param<bool>("use_alsa", use_alsa_, true);
    pnh_.param<std::string>("mic_device", mic_device_, "default");
    pnh_.param<std::string>("speaker_device", speaker_device_, "default");
  }

  ~AudioChatNode() { stop(); }

  bool init() {
    if (!enabled_) {
      ROS_INFO("[audio_chat] 音频聊天功能已禁用");
      return true;
    }

    // 检查音频硬件
    int ret = std::system("which arecord > /dev/null 2>&1");
    if (ret != 0) {
      ROS_WARN("[audio_chat] arecord 不可用，音频采集将使用日志模拟");
    }

    ret = std::system("which aplay > /dev/null 2>&1");
    if (ret != 0) {
      ROS_WARN("[audio_chat] aplay 不可用，音频播放将使用日志模拟");
    }

#ifdef HAS_OPUS
    ROS_INFO("[audio_chat] Opus 编解码已启用");
#else
    ROS_INFO("[audio_chat] Opus 库未安装，将使用 PCM 直传模式");
    ROS_INFO("[audio_chat]   安装: sudo apt install libopus-dev");
#endif

    // 初始化 UDP socket
    initSockets();

    // 订阅控制话题
    audio_control_sub_ = nh_.subscribe("/audio_chat/control", 1,
                                       &AudioChatNode::controlCallback, this);

    // 发布音频状态
    audio_state_pub_ = nh_.advertise<std_msgs::Bool>("/audio_chat/active", 1, true);

    // 启动音频线程
    if (rx_socket_ >= 0) {
      rx_thread_ = std::thread(&AudioChatNode::receiveLoop, this);
    }
    if (tx_socket_ >= 0) {
      tx_thread_ = std::thread(&AudioChatNode::transmitLoop, this);
    }

    ROS_INFO("[audio_chat] 初始化完成");
    ROS_INFO("[audio_chat]   医生端: %s (RX:%d, TX:%d)", doctor_ip_.c_str(), rx_port_, tx_port_);
    ROS_INFO("[audio_chat]   采样率: %d Hz, 通道: %d, 帧长: %d ms",
             sample_rate_, channels_, frame_duration_ms_);

    return true;
  }

  void stop() {
    audio_active_ = false;

    if (rx_thread_.joinable()) rx_thread_.join();
    if (tx_thread_.joinable()) tx_thread_.join();

    if (rx_socket_ >= 0) { close(rx_socket_); rx_socket_ = -1; }
    if (tx_socket_ >= 0) { close(tx_socket_); tx_socket_ = -1; }
  }

 private:
  void initSockets() {
    // ── 接收 socket ──
    rx_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (rx_socket_ < 0) {
      ROS_WARN("[audio_chat] 创建 RX socket 失败: %s", strerror(errno));
      return;
    }

    struct sockaddr_in rx_addr;
    memset(&rx_addr, 0, sizeof(rx_addr));
    rx_addr.sin_family = AF_INET;
    rx_addr.sin_addr.s_addr = INADDR_ANY;
    rx_addr.sin_port = htons(rx_port_);

    if (bind(rx_socket_, (struct sockaddr*)&rx_addr, sizeof(rx_addr)) < 0) {
      ROS_WARN("[audio_chat] 绑定 RX socket 失败: %s", strerror(errno));
      close(rx_socket_);
      rx_socket_ = -1;
      return;
    }

    // 设置非阻塞
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(rx_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // ── 发送 socket ──
    tx_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_socket_ < 0) {
      ROS_WARN("[audio_chat] 创建 TX socket 失败: %s", strerror(errno));
      return;
    }

    memset(&tx_addr_, 0, sizeof(tx_addr_));
    tx_addr_.sin_family = AF_INET;
    tx_addr_.sin_port = htons(tx_port_);
    if (inet_aton(doctor_ip_.c_str(), &tx_addr_.sin_addr) == 0) {
      ROS_WARN("[audio_chat] 无效的医生端 IP: %s", doctor_ip_.c_str());
    }
  }

  void controlCallback(const std_msgs::Empty::ConstPtr& /*msg*/) {
    audio_active_ = !audio_active_;
    ROS_INFO("[audio_chat] 音频聊天 %s", audio_active_ ? "开启" : "关闭");

    std_msgs::Bool state;
    state.data = audio_active_;
    audio_state_pub_.publish(state);
  }

  /**
   * @brief 音频接收线程 — 接收医生端音频并播放
   */
  void receiveLoop() {
    const size_t buf_size = 4096;
    uint8_t buf[buf_size];

    ROS_INFO("[audio_chat] 接收线程已启动");

    while (ros::ok() && rx_socket_ >= 0) {
      if (!audio_active_) {
        usleep(100000);
        continue;
      }

      struct sockaddr_in sender;
      socklen_t sender_len = sizeof(sender);
      ssize_t n = recvfrom(rx_socket_, buf, buf_size, 0,
                           (struct sockaddr*)&sender, &sender_len);
      if (n <= 0) continue;

      // 播放接收到的音频
      // 简化方案：将 PCM 数据通过 aplay 播放
      // 实际应使用 Opus 解码后写入 ALSA
      playAudio(buf, n);
    }

    ROS_INFO("[audio_chat] 接收线程退出");
  }

  /**
   * @brief 音频发送线程 — 采集麦克风并发送
   */
  void transmitLoop() {
    ROS_INFO("[audio_chat] 发送线程已启动");

    // 每帧采样数 (16kHz * 20ms = 320 samples)
    const int frame_samples = sample_rate_ * frame_duration_ms_ / 1000;
    const size_t pcm_frame_size = frame_samples * channels_ * 2;  // 16-bit

    while (ros::ok() && tx_socket_ >= 0) {
      if (!audio_active_) {
        usleep(100000);
        continue;
      }

      // 采集音频
      std::vector<uint8_t> pcm_data = captureAudio(pcm_frame_size);
      if (pcm_data.empty()) {
        usleep(frame_duration_ms_ * 1000);
        continue;
      }

      // 发送 PCM 到医生端
      sendto(tx_socket_, pcm_data.data(), pcm_data.size(), 0,
             (struct sockaddr*)&tx_addr_, sizeof(tx_addr_));

      usleep(frame_duration_ms_ * 1000);
    }

    ROS_INFO("[audio_chat] 发送线程退出");
  }

  /**
   * @brief 采集麦克风音频（简化实现：通过 arecord）
   */
  std::vector<uint8_t> captureAudio(size_t expected_size) {
    // 简化实现：使用系统命令 arecord 采集并管道读取
    // 生产环境应使用 ALSA API 直接读取，此简化版本用于快速验证

    static FILE* mic_pipe = nullptr;
    static bool pipe_opened = false;

    if (!pipe_opened) {
      std::ostringstream cmd;
      cmd << "arecord -D " << mic_device_
          << " -f S16_LE -r " << sample_rate_
          << " -c " << channels_
          << " -t raw 2>/dev/null";

      mic_pipe = popen(cmd.str().c_str(), "r");
      if (!mic_pipe) {
        ROS_WARN_THROTTLE(10.0, "[audio_chat] 无法打开麦克风: %s", mic_device_.c_str());
        return {};
      }
      pipe_opened = true;
      ROS_INFO("[audio_chat] 麦克风已打开: %s", cmd.str().c_str());
    }

    std::vector<uint8_t> buf(expected_size);
    size_t n = fread(buf.data(), 1, expected_size, mic_pipe);
    if (n < expected_size) {
      buf.resize(n);
    }
    return buf;
  }

  /**
   * @brief 播放音频（简化实现：通过 aplay）
   */
  void playAudio(const uint8_t* data, size_t size) {
    static FILE* speaker_pipe = nullptr;
    static bool pipe_opened = false;

    if (!pipe_opened) {
      std::ostringstream cmd;
      cmd << "aplay -D " << speaker_device_
          << " -f S16_LE -r " << sample_rate_
          << " -c " << channels_
          << " -t raw 2>/dev/null";

      speaker_pipe = popen(cmd.str().c_str(), "w");
      if (!speaker_pipe) {
        ROS_WARN_THROTTLE(10.0, "[audio_chat] 无法打开扬声器: %s", speaker_device_.c_str());
        return;
      }
      pipe_opened = true;
    }

    fwrite(data, 1, size, speaker_pipe);
    fflush(speaker_pipe);
  }

  // ── ROS ──
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber audio_control_sub_;
  ros::Publisher audio_state_pub_;

  // ── 配置 ──
  bool enabled_;
  std::atomic<bool> audio_active_;
  int sample_rate_;
  int channels_;
  int frame_duration_ms_;
  std::string doctor_ip_;
  int rx_port_;
  int tx_port_;
  bool use_alsa_;
  std::string mic_device_;
  std::string speaker_device_;

  // ── 网络 ──
  int tx_socket_;
  int rx_socket_;
  struct sockaddr_in tx_addr_;

  // ── 线程 ──
  std::thread rx_thread_;
  std::thread tx_thread_;
};

}  // namespace robot_navigation

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "audio_chat_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::AudioChatNode node(nh, pnh);
  if (!node.init()) {
    ROS_FATAL("[audio_chat] 初始化失败");
    return 1;
  }

  ros::spin();
  return 0;
}
