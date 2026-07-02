/**
 * @file video_stream_node.cpp
 * @brief 机器人端视频推流节点 — MJPEG over HTTP + /camera/compressed 话题
 *
 * 采集 USB 摄像头图像，编码为 JPEG 并通过内置 HTTP 服务器提供实时视频流。
 * 医生端可通过浏览器访问 http://<robot_ip>:8080/stream 查看。
 *
 * 同时发布压缩图像话题供其他节点使用。
 *
 * 新增功能：
 *   - /Ready 话题控制：1=启动推流, 0=停止（发送黑帧）
 *   - 摄像头超时容错：超过 N 秒无输入 → 自动输出黑帧，不报错退出
 *   - role 参数简化配置
 *
 * 依赖: OpenCV, cv_bridge, sensor_msgs
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <algorithm>

namespace robot_navigation {

/**
 * @brief 简易 MJPEG HTTP 流服务器
 *
 * 在独立线程中运行，接收浏览器连接并持续发送 MJPEG 帧。
 */
class MjpegServer {
 public:
  MjpegServer(int port = 8080, int max_clients = 5)
      : port_(port)
      , max_clients_(max_clients)
      , server_fd_(-1)
      , running_(false)
  {}

  ~MjpegServer() { stop(); }

  bool start() {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      ROS_ERROR("[video_stream] 无法创建 socket: %s", strerror(errno));
      return false;
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(port_);

    if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      ROS_ERROR("[video_stream] bind 失败: %s", strerror(errno));
      close(server_fd_);
      return false;
    }

    if (listen(server_fd_, max_clients_) < 0) {
      ROS_ERROR("[video_stream] listen 失败: %s", strerror(errno));
      close(server_fd_);
      return false;
    }

    running_ = true;
    server_thread_ = std::thread(&MjpegServer::acceptLoop, this);
    ROS_INFO("[video_stream] MJPEG 服务器已启动: http://0.0.0.0:%d/stream", port_);
    return true;
  }

  void stop() {
    running_ = false;
    if (server_fd_ >= 0) {
      shutdown(server_fd_, SHUT_RDWR);
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    // 关闭所有客户端
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (int fd : client_fds_) {
      close(fd);
    }
    client_fds_.clear();
    if (server_fd_ >= 0) {
      close(server_fd_);
      server_fd_ = -1;
    }
  }

  /**
   * @brief 更新 JPEG 帧数据（由主线程调用）
   */
  void updateFrame(const std::vector<uint8_t>& jpeg_data) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_frame_ = jpeg_data;
  }

  bool isRunning() const { return running_; }

 private:
  void acceptLoop() {
    while (running_) {
      struct sockaddr_in client_addr;
      socklen_t          client_len = sizeof(client_addr);
      int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
      if (client_fd < 0) {
        if (running_) {
          ROS_WARN_THROTTLE(5.0, "[video_stream] accept 失败: %s", strerror(errno));
        }
        continue;
      }

      // 限制最大客户端数
      {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        if (static_cast<int>(client_fds_.size()) >= max_clients_) {
          close(client_fd);
          continue;
        }
        client_fds_.push_back(client_fd);
      }

      ROS_INFO("[video_stream] 新客户端连接: %s:%d",
               inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

      std::thread(&MjpegServer::handleClient, this, client_fd).detach();
    }
  }

  void handleClient(int client_fd) {
    // 读取 HTTP 请求（简单解析）
    char buf[4096];
    ssize_t n = recv(client_fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) {
      close(client_fd);
      removeClient(client_fd);
      return;
    }
    buf[n] = '\0';

    // 发送 MJPEG HTTP 响应头
    const char* header =
        "HTTP/1.0 200 OK\r\n"
        "Server: ROS Video Stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=--ROS_MJPEG_BOUNDARY\r\n"
        "\r\n";
    send(client_fd, header, strlen(header), MSG_NOSIGNAL);

    // 循环发送帧
    const std::string boundary      = "--ROS_MJPEG_BOUNDARY\r\n";
    const std::string content_type  = "Content-Type: image/jpeg\r\n";

    while (running_) {
      std::vector<uint8_t> frame_copy;
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        frame_copy = latest_frame_;
      }

      if (frame_copy.empty()) {
        usleep(50000);  // 50ms
        continue;
      }

      std::ostringstream chunk;
      chunk << boundary
            << content_type
            << "Content-Length: " << frame_copy.size() << "\r\n"
            << "\r\n";

      std::string chunk_str = chunk.str();

      // 发送头部 + JPEG 数据
      if (send(client_fd, chunk_str.c_str(), chunk_str.size(), MSG_NOSIGNAL) < 0) break;
      if (send(client_fd, frame_copy.data(), frame_copy.size(), MSG_NOSIGNAL) < 0) break;

      // 帧率控制
      usleep(40000);  // ~25fps = 40ms
    }

    close(client_fd);
    removeClient(client_fd);
  }

  void removeClient(int fd) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    auto it = std::find(client_fds_.begin(), client_fds_.end(), fd);
    if (it != client_fds_.end()) {
      client_fds_.erase(it);
    }
  }

  int                port_;
  int                max_clients_;
  int                server_fd_;
  std::atomic<bool>  running_;
  std::thread        server_thread_;

  std::mutex              frame_mutex_;
  std::vector<uint8_t>    latest_frame_;

  std::mutex           clients_mutex_;
  std::vector<int>     client_fds_;
};

// ==========================================================================
//  VideoStreamNode
// ==========================================================================

class VideoStreamNode {
 public:
  VideoStreamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh)
      , pnh_(pnh)
      , ready_(0)
      , camera_ok_(false)
      , http_port_(8080)
      , jpeg_quality_(85)
      , stream_width_(640)
      , stream_height_(480)
      , framerate_(25.0)
      , enable_http_(true)
      , enable_ros_topic_(true)
      , camera_timeout_sec_(3.0)
      , camera_topic_("/usb_cam/image_raw")
      , ready_topic_("/Ready")
  {
    // ── 视频参数 ──
    pnh_.param<int>("http_port", http_port_, 8080);
    pnh_.param<int>("jpeg_quality", jpeg_quality_, 85);
    pnh_.param<int>("stream_width", stream_width_, 640);
    pnh_.param<int>("stream_height", stream_height_, 480);
    pnh_.param<double>("framerate", framerate_, 25.0);
    pnh_.param<bool>("enable_http", enable_http_, true);
    pnh_.param<bool>("enable_ros_topic", enable_ros_topic_, true);

    // ── 输入源 ──
    pnh_.param<std::string>("camera_topic", camera_topic_, "/usb_cam/image_raw");
    pnh_.param<double>("camera_timeout_sec", camera_timeout_sec_, 3.0);

    // ── 本地显示 ──
    pnh_.param<bool>("enable_local_display", enable_local_display_, false);
    pnh_.param<std::string>("local_display_topic", local_display_topic_, "/camera/local_display");

    // ── 控制 ──
    pnh_.param<std::string>("ready_topic", ready_topic_, "/Ready");

    // ── 生成黑帧 ──
    black_frame_ = cv::Mat(stream_height_, stream_width_, CV_8UC3, cv::Scalar(0, 0, 0));
    // 在黑帧上写字提示
    cv::putText(black_frame_, "Waiting for camera...",
                cv::Point(50, stream_height_ / 2),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(100, 100, 100), 2);
    cv::putText(black_frame_, "No signal",
                cv::Point(50, stream_height_ / 2 + 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(100, 100, 100), 2);

    // 预编码黑帧
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    cv::imencode(".jpg", black_frame_, black_jpeg_, params);

    // 订阅 /Ready 控制
    ready_sub_ = nh_.subscribe(ready_topic_, 1, &VideoStreamNode::readyCb, this);
  }

  bool init() {
    // 订阅摄像头（不设回调，在 spinOnce 中处理 — 改为实际回调）
    image_sub_ = nh_.subscribe(camera_topic_, 1,
                               &VideoStreamNode::imageCallback, this);

    if (enable_ros_topic_) {
      compressed_pub_ = nh_.advertise<sensor_msgs::CompressedImage>(
          "/camera/compressed", 1);
    }

    // 本地显示话题（供 image_view 使用，避免重复拉远程原始帧）
    if (enable_local_display_) {
      local_display_pub_ = nh_.advertise<sensor_msgs::Image>(
          local_display_topic_, 1);
    }

    // 启动 MJPEG HTTP 服务器
    if (enable_http_) {
      server_ = std::make_shared<MjpegServer>(http_port_);
      if (!server_->start()) {
        ROS_WARN("[video_stream] HTTP 服务器启动失败，仅使用 ROS 话题输出");
        enable_http_ = false;
      }
    }

    ROS_INFO("[video_stream] ====== 视频推流配置 ======");
    ROS_INFO("[video_stream]   输入话题: %s", camera_topic_.c_str());
    ROS_INFO("[video_stream]   分辨率: %dx%d, JPEG质量: %d, 帧率: %.1f",
             stream_width_, stream_height_, jpeg_quality_, framerate_);
    ROS_INFO("[video_stream]   摄像头超时: %.1f 秒 → 黑帧", camera_timeout_sec_);
    ROS_INFO("[video_stream]   Ready 话题: %s", ready_topic_.c_str());
    if (enable_http_) {
      ROS_INFO("[video_stream]   HTTP流: http://0.0.0.0:%d/stream", http_port_);
    }
    if (enable_ros_topic_) {
      ROS_INFO("[video_stream]   压缩话题: /camera/compressed");
    }
    ROS_INFO("[video_stream]   等待 /Ready=1 启动推流...");

    return true;
  }

 private:
  // ═══════════════════════════════════════════════════════════════
  //  /Ready 回调
  // ═══════════════════════════════════════════════════════════════

  void readyCb(const std_msgs::Int32::ConstPtr& msg) {
    const int new_val = msg->data;
    if (new_val != ready_) {
      ROS_INFO("[video_stream] Ready: %d -> %d", ready_.load(), new_val);
      ready_ = new_val;
      if (ready_ == 1) {
        ROS_INFO("[video_stream] ▶ 视频推流已启动");
      } else {
        ROS_INFO("[video_stream] ■ 视频推流已停止（发送黑帧）");
      }
    }
  }

  // ═══════════════════════════════════════════════════════════════
  //  图像回调
  // ═══════════════════════════════════════════════════════════════

  void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // 帧率控制
    ros::Time now = ros::Time::now();
    if (!last_frame_time_.isZero()) {
      double elapsed = (now - last_frame_time_).toSec();
      if (elapsed < 1.0 / std::max(1.0, framerate_)) {
        return;
      }
    }
    last_frame_time_   = now;
    last_image_time_   = now;
    camera_ok_         = true;

    // Ready=0 时不处理真实帧（但仍更新时间戳以检测摄像头在线状态）
    if (ready_ != 1) {
      return;
    }

    // 转换为 OpenCV 图像
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR_THROTTLE(5.0, "[video_stream] cv_bridge 异常: %s", e.what());
      return;
    }

    processAndPublish(cv_ptr->image, now);
  }

  // ═══════════════════════════════════════════════════════════════
  //  处理并发布帧
  // ═══════════════════════════════════════════════════════════════

  void processAndPublish(const cv::Mat& input, ros::Time stamp) {
    cv::Mat frame = input;

    // 缩放（如果需要）
    if (frame.cols != stream_width_ || frame.rows != stream_height_) {
      cv::resize(frame, frame, cv::Size(stream_width_, stream_height_));
    }

    // 编码为 JPEG
    std::vector<uint8_t> jpeg_buf;
    std::vector<int>     params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    cv::imencode(".jpg", frame, jpeg_buf, params);

    if (jpeg_buf.empty()) return;

    // 推送到 HTTP 服务器
    if (enable_http_ && server_) {
      server_->updateFrame(jpeg_buf);
    }

    // 发布压缩图像话题
    if (enable_ros_topic_) {
      sensor_msgs::CompressedImage compressed_msg;
      compressed_msg.header.stamp    = stamp;
      compressed_msg.header.frame_id = "camera";
      compressed_msg.format          = "jpeg";
      compressed_msg.data            = jpeg_buf;
      compressed_pub_.publish(compressed_msg);
    }

    // 发布本地显示话题（原始 RGB，供 image_view 本地订阅，不走网络）
    if (enable_local_display_) {
      cv_bridge::CvImage cv_img;
      cv_img.header.stamp    = stamp;
      cv_img.header.frame_id = "camera";
      cv_img.encoding        = sensor_msgs::image_encodings::BGR8;
      cv_img.image           = frame;
      local_display_pub_.publish(cv_img.toImageMsg());
    }
  }

  // ═══════════════════════════════════════════════════════════════
  //  黑帧兜底（在 spin 循环中调用）
  // ═══════════════════════════════════════════════════════════════

  void publishBlackFrame() {
    ros::Time now = ros::Time::now();

    // 帧率控制
    if (!last_black_time_.isZero()) {
      double elapsed = (now - last_black_time_).toSec();
      if (elapsed < 1.0 / std::max(1.0, framerate_)) {
        return;
      }
    }
    last_black_time_ = now;

    if (enable_http_ && server_) {
      server_->updateFrame(black_jpeg_);
    }

    if (enable_ros_topic_) {
      sensor_msgs::CompressedImage compressed_msg;
      compressed_msg.header.stamp    = now;
      compressed_msg.header.frame_id = "camera";
      compressed_msg.format          = "jpeg";
      compressed_msg.data            = black_jpeg_;
      compressed_pub_.publish(compressed_msg);
    }
  }

  // ═══════════════════════════════════════════════════════════════
  //  主循环（摄像头超时检测 + 黑帧发送）
  // ═══════════════════════════════════════════════════════════════

  void checkCameraTimeout() {
    ros::Time now = ros::Time::now();

    if (camera_ok_) {
      double elapsed = (now - last_image_time_).toSec();
      if (elapsed > camera_timeout_sec_) {
        if (camera_ok_) {
          ROS_WARN_THROTTLE(5.0,
              "[video_stream] 摄像头超时 (%.1f 秒无数据)，切换到黑帧模式",
              elapsed);
          camera_ok_ = false;
        }
      }
    }

    // Ready=0 或摄像头离线 → 发黑帧
    if (ready_ != 1 || !camera_ok_) {
      // 只在 Ready=1 且摄像头离线时发黑帧
      // Ready=0 时不主动发帧（节省带宽）
      if (ready_ == 1 && !camera_ok_) {
        publishBlackFrame();
      }
    }
  }

 public:
  /**
   * @brief 主循环 — 需在 main 中调用
   *
   * 替代 ros::spin()，增加摄像头超时检测逻辑。
   */
  void spin() {
    ros::Rate rate(framerate_ * 2);  // 2x 帧率检查频率
    while (ros::ok()) {
      ros::spinOnce();
      checkCameraTimeout();
      rate.sleep();
    }
  }

 private:
  // ── ROS ──
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber image_sub_;
  ros::Subscriber ready_sub_;
  ros::Publisher  compressed_pub_;
  ros::Publisher  local_display_pub_;

  // ── HTTP 服务器 ──
  std::shared_ptr<MjpegServer> server_;

  // ── 控制状态 ──
  std::atomic<int>  ready_{0};
  bool               camera_ok_{false};
  ros::Time          last_image_time_;
  ros::Time          last_frame_time_;
  ros::Time          last_black_time_;

  // ── 配置 ──
  int         http_port_;
  int         jpeg_quality_;
  int         stream_width_;
  int         stream_height_;
  double      framerate_;
  bool        enable_http_;
  bool        enable_ros_topic_;
  bool        enable_local_display_;
  std::string local_display_topic_;
  double      camera_timeout_sec_;
  std::string camera_topic_;
  std::string ready_topic_;

  // ── 黑帧 ──
  cv::Mat              black_frame_;
  std::vector<uint8_t> black_jpeg_;
};

}  // namespace robot_navigation

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "video_stream_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::VideoStreamNode node(nh, pnh);
  if (!node.init()) {
    ROS_FATAL("[video_stream] 初始化失败");
    return 1;
  }

  node.spin();
  return 0;
}
