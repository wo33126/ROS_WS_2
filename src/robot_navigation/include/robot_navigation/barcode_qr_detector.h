#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <robot_navigation/QrResult.h>
#include <std_msgs/String.h>

#include <zbar.h>

#include <atomic>
#include <mutex>
#include <string>

namespace robot_navigation {

/**
 * @brief 二维码/条形码检测节点
 *
 * 订阅摄像头图像，使用 zbar 库识别：
 * - 护士台二维码（两位数如 11, 13, 31, 33）→ 解析病床和药箱顺序
 * - 床头柜药品条形码 → 发布到对应病床话题
 *
 * 支持检测失败重试和超时机制。
 */
class BarcodeQrDetector {
 public:
  BarcodeQrDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~BarcodeQrDetector() = default;

  bool init();

 private:
  // ── 回调 ──
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

  // ── 辅助 ──
  bool parseQrCode(const std::string& data, QrResult& result);
  void publishEmptyQrResult();

  // ── ROS 接口 ──
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber      image_sub_;
  ros::Publisher       qr_result_pub_;
  ros::Publisher       barcode_bed1_pub_;
  ros::Publisher       barcode_bed3_pub_;
  ros::Publisher       display_text_pub_;

  // ── 状态 ──
  bool qr_detected_;
  bool barcode_bed1_detected_;
  bool barcode_bed3_detected_;

  ros::Time qr_scan_start_time_;
  ros::Time barcode_scan_start_time_;

  // ── 参数 ──
  double qr_timeout_sec_;         // QR 检测超时（秒）
  double barcode_timeout_sec_;    // 条形码检测超时（秒）
  int    image_queue_size_;
  std::string display_topic_;

  // ── zbar ──
  zbar::ImageScanner scanner_;

  // ── 线程安全 ──
  std::mutex mutex_;
};

}  // namespace robot_navigation
