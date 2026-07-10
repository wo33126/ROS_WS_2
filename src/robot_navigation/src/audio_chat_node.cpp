/**
 * @file audio_chat_node.cpp
 * @brief 双向音频通信节点 — 低延迟 Opus/UDP 方案
 *
 * 提供医生端与病人端的双向音频交流功能：
 *   1. 麦克风采集 (arecord) → Opus 编码 → UDP 发送给对端
 *   2. UDP 接收 ← 对端音频 → Opus 解码 → 扬声器播放 (aplay)
 *   3. 受 /Ready 话题控制：1=启动通话, 0=停止
 *   4. role 参数自动配置 IP 和端口（master/slave）
 *
 * 协议：
 *   - 音频格式: 48kHz, 16-bit, mono PCM → Opus 编码 (32kbps)
 *   - 传输: UDP 单播（机器人 ↔ 医生端 PC）
 *   - 端口: 双方均 RX=5004, TX=5004
 *
 * 依赖: libopus (libopus-dev)
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <sstream>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstdlib>
#include <cstdio>

// ── Opus 编解码（条件编译）──
#ifdef HAS_OPUS
#include <opus/opus.h>
#endif

namespace robot_navigation {

/**
 * @brief 音频聊天节点
 *
 * 管理双向音频流：
 *   - 音频采集线程: arecord 管道读取 PCM → Opus 编码 → UDP 发送
 *   - 音频接收线程: UDP 接收 → Opus 解码 → aplay 管道写入
 *   - 受 /Ready 话题控制启停
 */
class AudioChatNode {
 public:
  AudioChatNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh)
      , pnh_(pnh)
      , ready_(0)
      , audio_active_(true)
      , role_("master")
      , sample_rate_(48000)
      , channels_(1)
      , frame_duration_ms_(20)
      , opus_bitrate_(32000)
      , opus_complexity_(5)
      , local_ip_("192.168.15.128")
      , remote_ip_("192.168.15.129")
      , rx_port_(5004)
      , tx_port_(5004)
      , mic_device_("default")
      , speaker_device_("default")
      , tx_socket_(-1)
      , rx_socket_(-1)
      , ready_topic_("/Ready")
#ifdef HAS_OPUS
      , encoder_(nullptr)
      , decoder_(nullptr)
#endif
  {
    // ── 角色参数（master/slave）──
    pnh_.param<std::string>("role", role_, "master");
    applyRoleDefaults();

    // ── 允许显式覆盖 IP/端口 ──
    pnh_.param<std::string>("remote_ip", remote_ip_, remote_ip_);
    pnh_.param<std::string>("local_ip", local_ip_, local_ip_);
    pnh_.param<int>("rx_port", rx_port_, 5004);
    pnh_.param<int>("tx_port", tx_port_, 5004);

    // ── 音频参数 ──
    pnh_.param<int>("sample_rate", sample_rate_, 48000);
    pnh_.param<int>("channels", channels_, 1);
    pnh_.param<int>("frame_duration_ms", frame_duration_ms_, 20);
    pnh_.param<int>("opus_bitrate", opus_bitrate_, 32000);
    pnh_.param<int>("opus_complexity", opus_complexity_, 5);

    // ── 设备 ──
    pnh_.param<std::string>("mic_device", mic_device_, "default");
    pnh_.param<std::string>("speaker_device", speaker_device_, "default");

    // ── Ready 话题 ──
    pnh_.param<std::string>("ready_topic", ready_topic_, "/Ready");

    // 订阅 /Ready 控制话题
    ready_sub_ = nh_.subscribe(ready_topic_, 1, &AudioChatNode::readyCb, this);
  }

  ~AudioChatNode() { stop(); }

  bool init() {
    // 检查音频工具
    int ret = std::system("which arecord > /dev/null 2>&1");
    if (ret != 0) {
      ROS_WARN("[audio_chat] arecord 不可用，音频采集将无法工作");
    }
    ret = std::system("which aplay > /dev/null 2>&1");
    if (ret != 0) {
      ROS_WARN("[audio_chat] aplay 不可用，音频播放将无法工作");
    }

#ifdef HAS_OPUS
    // Opus 编码器: 48kHz, mono, 低延迟音乐/VoIP 模式
    int opus_err = 0;
    encoder_ = opus_encoder_create(sample_rate_, channels_, OPUS_APPLICATION_VOIP, &opus_err);
    if (opus_err != OPUS_OK || !encoder_) {
      ROS_ERROR("[audio_chat] Opus 编码器创建失败: %d", opus_err);
    } else {
      opus_encoder_ctl(encoder_, OPUS_SET_BITRATE(opus_bitrate_));
      opus_encoder_ctl(encoder_, OPUS_SET_COMPLEXITY(opus_complexity_));
      opus_encoder_ctl(encoder_, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
      ROS_INFO("[audio_chat] Opus 编码器已就绪: %d Hz, %d bps, complexity=%d",
               sample_rate_, opus_bitrate_, opus_complexity_);
    }

    // Opus 解码器
    decoder_ = opus_decoder_create(sample_rate_, channels_, &opus_err);
    if (opus_err != OPUS_OK || !decoder_) {
      ROS_ERROR("[audio_chat] Opus 解码器创建失败: %d", opus_err);
    } else {
      ROS_INFO("[audio_chat] Opus 解码器已就绪");
    }
#else
    ROS_INFO("[audio_chat] Opus 未编译，使用 PCM 直传模式");
    ROS_INFO("[audio_chat]   安装: sudo apt install libopus-dev 后重新编译");
#endif

    // 服务状态话题
    audio_state_pub_ = nh_.advertise<std_msgs::Bool>("/audio_chat/active", 1, true);

    // 打印配置
    ROS_INFO("[audio_chat] ====== 音频聊天配置 ======");
    ROS_INFO("[audio_chat]   角色: %s", role_.c_str());
    ROS_INFO("[audio_chat]   本机 IP: %s", local_ip_.c_str());
    ROS_INFO("[audio_chat]   对端 IP: %s", remote_ip_.c_str());
    ROS_INFO("[audio_chat]   RX 端口: %d, TX 端口: %d", rx_port_, tx_port_);
    ROS_INFO("[audio_chat]   采样率: %d Hz, 通道: %d", sample_rate_, channels_);
    ROS_INFO("[audio_chat]   帧长: %d ms, 码率: %d bps", frame_duration_ms_, opus_bitrate_);
    ROS_INFO("[audio_chat]   麦克风: %s, 扬声器: %s", mic_device_.c_str(), speaker_device_.c_str());
    ROS_INFO("[audio_chat]   Ready 话题: %s", ready_topic_.c_str());
    ROS_INFO("[audio_chat]   等待 /Ready=1 启动通话...");

    return true;
  }

  void stop() {
    ready_ = 0;
    stopping_ = true;

    // 等待线程结束
    if (rx_thread_.joinable()) rx_thread_.join();
    if (tx_thread_.joinable()) tx_thread_.join();

    // 关闭 socket
    if (rx_socket_ >= 0) { close(rx_socket_); rx_socket_ = -1; }
    if (tx_socket_ >= 0) { close(tx_socket_); tx_socket_ = -1; }

    // 销毁 Opus 编解码器
#ifdef HAS_OPUS
    if (encoder_) { opus_encoder_destroy(encoder_); encoder_ = nullptr; }
    if (decoder_) { opus_decoder_destroy(decoder_); decoder_ = nullptr; }
#endif

    // 发布状态
    publishState(false);
  }

  /**
   * @brief 主循环 — 响应 /Ready 变化，管理线程生命周期
   */
  void spin() {
    ros::Rate rate(20.0);
    while (ros::ok() && !stopping_) {
      if (ready_ == 1 && !threads_running_) {
        startStreaming();
      } else if (ready_ == 0 && threads_running_) {
        stopStreaming();
      }

      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  // ═══════════════════════════════════════════════════════════════
  //  角色默认配置
  // ═══════════════════════════════════════════════════════════════

  void applyRoleDefaults() {
    if (role_ == "slave") {
      local_ip_  = "192.168.15.129";
      remote_ip_ = "192.168.15.128";
    } else {
      // master (default)
      role_      = "master";
      local_ip_  = "192.168.15.128";
      remote_ip_ = "192.168.15.129";
    }
  }

  // ═══════════════════════════════════════════════════════════════
  //  /Ready 回调
  // ═══════════════════════════════════════════════════════════════

  void readyCb(const std_msgs::Int32::ConstPtr& msg) {
    const int new_val = msg->data;
    if (new_val != ready_) {
      ROS_INFO("[audio_chat] Ready: %d -> %d", ready_.load(), new_val);
      ready_ = new_val;
    }
  }

  // ═══════════════════════════════════════════════════════════════
  //  流启动 / 停止
  // ═══════════════════════════════════════════════════════════════

  void startStreaming() {
    if (threads_running_) return;

    // 初始化 UDP socket
    initSockets();
    if (rx_socket_ < 0 && tx_socket_ < 0) {
      ROS_ERROR("[audio_chat] Socket 初始化失败，无法启动音频流");
      return;
    }

    threads_running_ = true;
    audio_active_    = true;

    // 启动收发线程
    if (rx_socket_ >= 0) {
      rx_thread_ = std::thread(&AudioChatNode::receiveLoop, this);
    }
    if (tx_socket_ >= 0) {
      tx_thread_ = std::thread(&AudioChatNode::transmitLoop, this);
    }

    publishState(true);
    ROS_INFO("[audio_chat] ▶ 音频通话已启动");
  }

  void stopStreaming() {
    if (!threads_running_) return;

    ROS_INFO("[audio_chat] ■ 正在停止音频通话...");
    audio_active_    = false;
    threads_running_ = false;

    // 等待线程退出
    if (rx_thread_.joinable()) rx_thread_.join();
    if (tx_thread_.joinable()) tx_thread_.join();

    // 清理 socket（线程已退出，安全关闭）
    if (rx_socket_ >= 0) { close(rx_socket_); rx_socket_ = -1; }
    if (tx_socket_ >= 0) { close(tx_socket_); tx_socket_ = -1; }

    // 关闭音频管道
    closeAudioPipes();

    publishState(false);
    ROS_INFO("[audio_chat] ■ 音频通话已停止");
  }

  void publishState(bool active) {
    std_msgs::Bool state;
    state.data = active;
    audio_state_pub_.publish(state);
  }

  // ═══════════════════════════════════════════════════════════════
  //  UDP Socket
  // ═══════════════════════════════════════════════════════════════

  void initSockets() {
    // ── 接收 socket ──
    rx_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (rx_socket_ < 0) {
      ROS_ERROR("[audio_chat] 创建 RX socket 失败: %s", strerror(errno));
      return;
    }

    // 设置接收缓冲区 (64KB)
    int rcvbuf = 65536;
    setsockopt(rx_socket_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    // 设置接收超时 100ms
    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 100000;
    setsockopt(rx_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in rx_addr;
    memset(&rx_addr, 0, sizeof(rx_addr));
    rx_addr.sin_family      = AF_INET;
    rx_addr.sin_addr.s_addr = INADDR_ANY;
    rx_addr.sin_port        = htons(rx_port_);

    if (bind(rx_socket_, (struct sockaddr*)&rx_addr, sizeof(rx_addr)) < 0) {
      ROS_ERROR("[audio_chat] 绑定 RX socket 失败 (端口 %d): %s", rx_port_, strerror(errno));
      close(rx_socket_);
      rx_socket_ = -1;
      return;
    }

    ROS_INFO("[audio_chat] RX socket 已绑定 0.0.0.0:%d", rx_port_);

    // ── 发送 socket ──
    tx_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_socket_ < 0) {
      ROS_ERROR("[audio_chat] 创建 TX socket 失败: %s", strerror(errno));
      return;
    }

    // 设置发送缓冲区
    int sndbuf = 65536;
    setsockopt(tx_socket_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    memset(&tx_addr_, 0, sizeof(tx_addr_));
    tx_addr_.sin_family = AF_INET;
    tx_addr_.sin_port   = htons(tx_port_);
    if (inet_aton(remote_ip_.c_str(), &tx_addr_.sin_addr) == 0) {
      ROS_ERROR("[audio_chat] 无效的对端 IP: %s", remote_ip_.c_str());
      close(tx_socket_);
      tx_socket_ = -1;
      return;
    }

    ROS_INFO("[audio_chat] TX socket 目标: %s:%d", remote_ip_.c_str(), tx_port_);
  }

  // ═══════════════════════════════════════════════════════════════
  //  音频采集 / 播放管道
  // ═══════════════════════════════════════════════════════════════

  /**
   * @brief 打开麦克风采集管道
   *
   * stdbuf -i0 -o0 强制 pacat 的 stdout 无缓冲，消除内核管道积压延迟
   * latency-msec=20 请求最小采集片段（PA 硬件下限约 100ms，但请求越小越好）
   */
  FILE* openMic() {
    std::ostringstream cmd;
    // stdbuf -i0 -o0: 禁用 pacat 进程自身的 stdio 缓冲，PCM 数据立即写入管道
    cmd << "stdbuf -i0 -o0 pacat --record"
        << " --format=s16le"
        << " --rate=" << sample_rate_
        << " --channels=" << channels_
        << " --latency-msec=50";
    if (mic_device_ != "default") {
      cmd << " --device=" << mic_device_;
    }
    cmd << " 2>&1";

    ROS_INFO("[audio_chat] 打开麦克风: %s", cmd.str().c_str());

    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (!pipe) {
      ROS_ERROR("[audio_chat] 无法打开麦克风设备: %s", mic_device_.c_str());
    }
    if (pipe) setvbuf(pipe, NULL, _IONBF, 0);
    return pipe;
  }

  /**
   * @brief 打开扬声器播放管道
   *
   * latency-msec=100 作为抖动缓冲（VM 内网抖动 < 50ms，100ms 够用）
   * PA 硬件固定延迟 ~100ms，播放侧总延迟约 200ms
   */
  FILE* openSpeaker() {
    std::ostringstream cmd;
    cmd << "pacat --playback"
        << " --format=s16le"
        << " --rate=" << sample_rate_
        << " --channels=" << channels_
        << " --latency-msec=100";
    if (speaker_device_ != "default") {
      cmd << " --device=" << speaker_device_;
    }
    cmd << " 2>&1";

    ROS_INFO("[audio_chat] 打开扬声器: %s", cmd.str().c_str());

    FILE* pipe = popen(cmd.str().c_str(), "w");
    if (!pipe) {
      ROS_ERROR("[audio_chat] 无法打开扬声器设备: %s", speaker_device_.c_str());
    }
    // 禁用 stdio 缓冲，降低延迟
    if (pipe) setvbuf(pipe, NULL, _IONBF, 0);
    return pipe;
  }

  void closePipe(FILE*& pipe) {
    if (pipe) {
      pclose(pipe);
      pipe = nullptr;
    }
  }

  void closeAudioPipes() {
    closePipe(mic_pipe_);
    closePipe(speaker_pipe_);
  }

  // ═══════════════════════════════════════════════════════════════
  //  接收线程 — UDP → Opus解码 → aplay
  // ═══════════════════════════════════════════════════════════════

  void receiveLoop() {
    ROS_INFO("[audio_chat] 接收线程已启动");

    // 打开扬声器管道
    speaker_pipe_ = openSpeaker();
    if (!speaker_pipe_) {
      ROS_ERROR("[audio_chat] 扬声器打开失败，接收线程退出");
      return;
    }

    const size_t buf_size           = 4096;  // UDP 最大包 + Opus 帧
    uint8_t      buf[buf_size];
    const int    frame_samples      = sample_rate_ * frame_duration_ms_ / 1000;

    // 每 2ms 轮询一次（低延迟关键优化点）
    const int poll_interval_us = 2000;

    // 心跳计数器
    int      pkt_count    = 0;
    int      decode_errs  = 0;
    ros::Time last_report = ros::Time::now();

    while (ros::ok() && threads_running_ && rx_socket_ >= 0) {
      // 检查 speaker 管道是否断开
      if (feof(speaker_pipe_) || ferror(speaker_pipe_)) {
        ROS_ERROR("[audio_chat] 扬声器管道断开! 接收线程退出");
        closePipe(speaker_pipe_);
        return;
      }

      // 检查 socket 是否有数据
      struct sockaddr_in sender;
      socklen_t sender_len = sizeof(sender);
      ssize_t n = recvfrom(rx_socket_, buf, buf_size, MSG_DONTWAIT,
                           (struct sockaddr*)&sender, &sender_len);

      if (n <= 0) {
        usleep(poll_interval_us);
        continue;
      }

      // 解码并播放
#ifdef HAS_OPUS
      if (decoder_) {
        // Opus 解码 → PCM
        std::vector<int16_t> pcm_out(frame_samples);
        int decoded = opus_decode(decoder_,
                                  buf, static_cast<opus_int32>(n),
                                  pcm_out.data(), frame_samples, 0);
        if (decoded > 0) {
          size_t bytes = decoded * channels_ * sizeof(int16_t);
          fwrite(pcm_out.data(), 1, bytes, speaker_pipe_);
          fflush(speaker_pipe_);
          pkt_count++;
        } else if (decoded < 0) {
          decode_errs++;
          if (decode_errs <= 3) {
            ROS_WARN("[audio_chat] Opus 解码错误: %d (包大小=%ld)", decoded, n);
          }
        }
      }
#else
      // PCM 直通模式
      size_t pcm_size = (n / (channels_ * sizeof(int16_t))) * channels_ * sizeof(int16_t);
      if (pcm_size > 0) {
        fwrite(buf, 1, pcm_size, speaker_pipe_);
        fflush(speaker_pipe_);
        pkt_count++;
      }
#endif

      // 心跳日志（每 5 秒）
      if ((ros::Time::now() - last_report).toSec() >= 5.0) {
        ROS_INFO("[audio_chat] RX 心跳: %d 包已接收, %d 解码错误",
                 pkt_count, decode_errs);
        last_report = ros::Time::now();
      }
    }

    closePipe(speaker_pipe_);
    ROS_INFO("[audio_chat] 接收线程退出");
  }

  // ═══════════════════════════════════════════════════════════════
  //  发送线程 — arecord → Opus编码 → UDP
  // ═══════════════════════════════════════════════════════════════

  void transmitLoop() {
    ROS_INFO("[audio_chat] 发送线程已启动");

    // 打开麦克风管道
    mic_pipe_ = openMic();
    if (!mic_pipe_) {
      ROS_ERROR("[audio_chat] 麦克风打开失败，发送线程退出");
      return;
    }

    const int    frame_samples    = sample_rate_ * frame_duration_ms_ / 1000;
    const size_t pcm_frame_bytes  = frame_samples * channels_ * sizeof(int16_t);
    std::vector<uint8_t> pcm_buf(pcm_frame_bytes);

    // fread 本身会阻塞到 ALSA 填满一帧（~20ms），不需要额外 sleep 来限速
    // 保留一个极短让步，避免 CPU 空转（仅在 fread 意外快返回时起作用）

    // 心跳计数器
    int      frame_count  = 0;
    int      error_count  = 0;
    ros::Time last_report = ros::Time::now();

    while (ros::ok() && threads_running_ && tx_socket_ >= 0) {
      // 读取一帧 PCM 数据
      size_t total_read = 0;
      size_t remaining  = pcm_frame_bytes;
      uint8_t* ptr      = pcm_buf.data();

      while (remaining > 0 && threads_running_) {
        // 检查管道是否断开
        if (feof(mic_pipe_) || ferror(mic_pipe_)) {
          ROS_ERROR("[audio_chat] 麦克风管道断开! 发送线程退出");
          closePipe(mic_pipe_);
          return;
        }
        size_t n = fread(ptr, 1, remaining, mic_pipe_);
        if (n == 0) {
          usleep(1000);
          continue;
        }
        ptr        += n;
        total_read += n;
        remaining  -= n;
      }

      if (!threads_running_) break;
      if (total_read < pcm_frame_bytes) continue;

      // 编码并发送
#ifdef HAS_OPUS
      if (encoder_) {
        const int16_t* pcm_samples = reinterpret_cast<const int16_t*>(pcm_buf.data());
        uint8_t        opus_pkt[1024];

        int encoded = opus_encode(encoder_, pcm_samples, frame_samples,
                                  opus_pkt, sizeof(opus_pkt));
        if (encoded > 0) {
          ssize_t sent = sendto(tx_socket_, opus_pkt, encoded, MSG_DONTWAIT,
                                (struct sockaddr*)&tx_addr_, sizeof(tx_addr_));
          if (sent > 0) {
            frame_count++;
          } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
            error_count++;
          }
        } else if (encoded < 0) {
          if (error_count++ % 100 == 0) {
            ROS_WARN_THROTTLE(5.0, "[audio_chat] Opus 编码错误: %d", encoded);
          }
        }
      }
#else
      ssize_t sent = sendto(tx_socket_, pcm_buf.data(), pcm_frame_bytes, MSG_DONTWAIT,
                            (struct sockaddr*)&tx_addr_, sizeof(tx_addr_));
      if (sent > 0) frame_count++;
#endif

      // 心跳日志（每 5 秒）
      if ((ros::Time::now() - last_report).toSec() >= 5.0) {
        ROS_INFO("[audio_chat] TX 心跳: %d 帧已发送, %d 错误, 目标 %s:%d",
                 frame_count, error_count, remote_ip_.c_str(), tx_port_);
        last_report = ros::Time::now();
      }

      // 不 sleep：fread 已经天然限速到一帧周期（~20ms）
    }

    closePipe(mic_pipe_);
    ROS_INFO("[audio_chat] 发送线程退出 (共发送 %d 帧)", frame_count);
  }

  // ═══════════════════════════════════════════════════════════════
  //  成员变量
  // ═══════════════════════════════════════════════════════════════

  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber ready_sub_;
  ros::Publisher  audio_state_pub_;

  // 控制状态
  std::atomic<int>  ready_{0};
  std::atomic<bool> audio_active_{true};
  std::atomic<bool> threads_running_{false};
  std::atomic<bool> stopping_{false};

  // 配置
  std::string role_;
  int         sample_rate_;
  int         channels_;
  int         frame_duration_ms_;
  int         opus_bitrate_;
  int         opus_complexity_;
  std::string local_ip_;
  std::string remote_ip_;
  int         rx_port_;
  int         tx_port_;
  std::string mic_device_;
  std::string speaker_device_;

  // 网络
  int                tx_socket_{-1};
  int                rx_socket_{-1};

  std::string ready_topic_;
  struct sockaddr_in tx_addr_;

  // 音频管道
  FILE* mic_pipe_{nullptr};
  FILE* speaker_pipe_{nullptr};

  // 线程
  std::thread rx_thread_;
  std::thread tx_thread_;

  // Opus
#ifdef HAS_OPUS
  OpusEncoder* encoder_;
  OpusDecoder* decoder_;
#endif
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

  // 使用 spin() 而非 ros::spin()，因为需要主动管理线程生命周期
  node.spin();
  node.stop();
  return 0;
}
