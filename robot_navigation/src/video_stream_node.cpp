/**
 * @file video_stream_node.cpp
 * @brief 机器人端视频推流节点 — MJPEG over HTTP
 *
 * 采集 USB 摄像头图像，编码为 MJPEG 并通过内置 HTTP 服务器提供实时视频流。
 * 医生端可通过浏览器访问 http://<robot_ip>:8080/stream 查看。
 *
 * 同时发布压缩图像话题供其他节点使用。
 *
 * 依赖: OpenCV, cv_bridge, sensor_msgs
 */

#include <ros/ros.h>
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
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port_);

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

 private:
  void acceptLoop() {
    while (running_) {
      struct sockaddr_in client_addr;
      socklen_t client_len = sizeof(client_addr);
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
    const std::string boundary = "--ROS_MJPEG_BOUNDARY\r\n";
    const std::string content_type = "Content-Type: image/jpeg\r\n";

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

      // 帧率控制（约 15fps）
      usleep(66000);
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

  int port_;
  int max_clients_;
  int server_fd_;
  std::atomic<bool> running_;
  std::thread server_thread_;

  std::mutex frame_mutex_;
  std::vector<uint8_t> latest_frame_;

  std::mutex clients_mutex_;
  std::vector<int> client_fds_;
};

// ==========================================================================
//  VideoStreamNode
// ==========================================================================

class VideoStreamNode {
 public:
  VideoStreamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh)
      , pnh_(pnh)
      , http_port_(8080)
      , jpeg_quality_(70)
      , stream_width_(640)
      , stream_height_(480)
      , framerate_(15.0)
      , enable_http_(true)
      , enable_ros_topic_(true)
      , camera_topic_("/usb_cam/image_raw")
  {
    pnh_.param<int>("http_port", http_port_, 8080);
    pnh_.param<int>("jpeg_quality", jpeg_quality_, 70);
    pnh_.param<int>("stream_width", stream_width_, 640);
    pnh_.param<int>("stream_height", stream_height_, 480);
    pnh_.param<double>("framerate", framerate_, 15.0);
    pnh_.param<bool>("enable_http", enable_http_, true);
    pnh_.param<bool>("enable_ros_topic", enable_ros_topic_, true);
    pnh_.param<std::string>("camera_topic", camera_topic_, "/usb_cam/image_raw");
  }

  bool init() {
    image_sub_ = nh_.subscribe(camera_topic_, 1,
                               &VideoStreamNode::imageCallback, this);

    if (enable_ros_topic_) {
      compressed_pub_ = nh_.advertise<sensor_msgs::CompressedImage>(
          "/camera/compressed", 1);
    }

    // 启动 MJPEG HTTP 服务器
    if (enable_http_) {
      server_ = std::make_shared<MjpegServer>(http_port_);
      if (!server_->start()) {
        ROS_WARN("[video_stream] HTTP 服务器启动失败，仅使用 ROS 话题输出");
        enable_http_ = false;
      }
    }

    ROS_INFO("[video_stream] 初始化完成");
    ROS_INFO("[video_stream]   摄像头: %s", camera_topic_.c_str());
    ROS_INFO("[video_stream]   分辨率: %dx%d, JPEG质量: %d, 帧率: %.1f",
             stream_width_, stream_height_, jpeg_quality_, framerate_);
    if (enable_http_) {
      ROS_INFO("[video_stream]   HTTP流: http://0.0.0.0:%d/stream", http_port_);
    }
    if (enable_ros_topic_) {
      ROS_INFO("[video_stream]   压缩话题: /camera/compressed");
    }

    return true;
  }

 private:
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // 帧率控制
    ros::Time now = ros::Time::now();
    if (!last_frame_time_.isZero()) {
      double elapsed = (now - last_frame_time_).toSec();
      if (elapsed < 1.0 / std::max(1.0, framerate_)) {
        return;
      }
    }
    last_frame_time_ = now;

    // 转换为 OpenCV 图像
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR_THROTTLE(5.0, "[video_stream] cv_bridge 异常: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;

    // 缩放（如果需要）
    if (frame.cols != stream_width_ || frame.rows != stream_height_) {
      cv::resize(frame, frame, cv::Size(stream_width_, stream_height_));
    }

    // 编码为 JPEG
    std::vector<uint8_t> jpeg_buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    cv::imencode(".jpg", frame, jpeg_buf, params);

    if (jpeg_buf.empty()) return;

    // 推送到 HTTP 服务器
    if (enable_http_ && server_) {
      server_->updateFrame(jpeg_buf);
    }

    // 发布压缩图像话题
    if (enable_ros_topic_) {
      sensor_msgs::CompressedImage compressed_msg;
      compressed_msg.header.stamp = now;
      compressed_msg.header.frame_id = "camera";
      compressed_msg.format = "jpeg";
      compressed_msg.data = jpeg_buf;
      compressed_pub_.publish(compressed_msg);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber image_sub_;
  ros::Publisher compressed_pub_;

  std::shared_ptr<MjpegServer> server_;

  int http_port_;
  int jpeg_quality_;
  int stream_width_;
  int stream_height_;
  double framerate_;
  bool enable_http_;
  bool enable_ros_topic_;
  std::string camera_topic_;
  ros::Time last_frame_time_;
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

  ros::spin();
  return 0;
}
