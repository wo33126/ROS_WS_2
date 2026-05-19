/**
 * @file barcode_qr_detector.cpp
 * @brief 二维码/条形码检测节点实现
 *
 * 订阅摄像头图像，使用 zbar 库识别二维码和条形码。
 * - 护士台二维码（两位数）→ 解析病床/药箱 → /qr_result
 * - 床头柜药品条形码 → /barcode_bed1 或 /barcode_bed3
 */

#include "robot_navigation/barcode_qr_detector.h"

#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>

namespace robot_navigation {

// ── 构造 ───────────────────────────────────────────────────
BarcodeQrDetector::BarcodeQrDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , qr_detected_(false)
    , barcode_bed1_detected_(false)
    , barcode_bed3_detected_(false)
    , qr_timeout_sec_(30.0)
    , barcode_timeout_sec_(30.0)
    , image_queue_size_(10)
    , display_topic_("/robot_display")
{
  // 配置 zbar 扫描器：启用 QR 码和所有条形码格式
  scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

  pnh_.param<double>("qr_timeout_sec", qr_timeout_sec_, 30.0);
  pnh_.param<double>("barcode_timeout_sec", barcode_timeout_sec_, 30.0);
  pnh_.param<int>("image_queue_size", image_queue_size_, 10);
  pnh_.param<std::string>("display_topic", display_topic_, "/robot_display");
}

bool BarcodeQrDetector::init() {
  // ── 订阅摄像头图像 ──
  image_sub_ = nh_.subscribe("/usb_cam/image_raw", image_queue_size_,
                             &BarcodeQrDetector::imageCallback, this);

  // ── 发布者 ──
  qr_result_pub_     = nh_.advertise<robot_navigation::QrResult>("/qr_result", 1, true);
  barcode_bed1_pub_  = nh_.advertise<std_msgs::String>("/barcode_bed1", 1, true);
  barcode_bed3_pub_  = nh_.advertise<std_msgs::String>("/barcode_bed3", 1, true);
  display_text_pub_  = nh_.advertise<std_msgs::String>(display_topic_, 1, true);

  // ── 初始化时间戳 ──
  qr_scan_start_time_ = ros::Time::now();

  ROS_INFO("[barcode_qr_detector] 初始化完成");
  ROS_INFO("[barcode_qr_detector]   订阅: /usb_cam/image_raw");
  ROS_INFO("[barcode_qr_detector]   发布: /qr_result, /barcode_bed1, /barcode_bed3");
  ROS_INFO("[barcode_qr_detector]   QR超时: %.0f s, 条形码超时: %.0f s",
           qr_timeout_sec_, barcode_timeout_sec_);

  return true;
}

// ── 图像回调 ───────────────────────────────────────────────
void BarcodeQrDetector::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  // ── 超时检查 ──
  const double qr_elapsed = (ros::Time::now() - qr_scan_start_time_).toSec();
  if (!qr_detected_ && qr_elapsed > qr_timeout_sec_) {
    ROS_WARN("[barcode_qr_detector] QR 检测超时 (%.1f s)，发布空结果", qr_timeout_sec_);
    publishEmptyQrResult();
    qr_detected_ = true;  // 标记"已处理"避免重复发布
  }

  // ── 转换 ROS 图像 → OpenCV ──
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_THROTTLE(5.0, "[barcode_qr_detector] cv_bridge 异常: %s", e.what());
    return;
  }

  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

  // ── zbar 扫描 ──
  zbar::Image zbar_image(gray.cols, gray.rows, "Y800",
                         gray.data, gray.cols * gray.rows);
  int n = scanner_.scan(zbar_image);

  if (n <= 0) return;

  // ── 遍历检测结果 ──
  for (auto it = zbar_image.symbol_begin(); it != zbar_image.symbol_end(); ++it) {
    std::string data = it->get_data();
    zbar::zbar_symbol_type_t type = it->get_type();

    ROS_INFO("[barcode_qr_detector] 检测到: type=%s, data=\"%s\"",
             zbar::zbar_get_symbol_name(type), data.c_str());

    if (!qr_detected_ && type == zbar::ZBAR_QRCODE) {
      // ── 处理 QR 码（护士台二维码） ──
      robot_navigation::QrResult result;
      if (parseQrCode(data, result)) {
        qr_result_pub_.publish(result);
        qr_detected_ = true;
        ROS_INFO("[barcode_qr_detector] QR 结果已发布: "
                 "first_bed=%d, first_box=%d, second_bed=%d, second_box=%d",
                 result.first_bed, result.first_box,
                 result.second_bed, result.second_box);
      } else {
        ROS_WARN("[barcode_qr_detector] QR 码内容非法: \"%s\"（应为两位数如 11,13,31,33）",
                 data.c_str());
      }
    }

    if (type == zbar::ZBAR_EAN13 || type == zbar::ZBAR_EAN8 ||
        type == zbar::ZBAR_UPCA  || type == zbar::ZBAR_UPCE  ||
        type == zbar::ZBAR_CODE128 || type == zbar::ZBAR_CODE39 ||
        type == zbar::ZBAR_I25) {
      // ── 处理条形码（药品条形码） ──
      std_msgs::String barcode_msg;
      barcode_msg.data = data;

      // 发布到 /barcode_bed1 和 /barcode_bed3 —— 实际分配由 mission_controller 根据
      // 当前上下文决定，这里我们同时发布两个话题；mission_controller 按需取用。
      // 如果已有一个检测到，则分配给另一个话题。
      if (!barcode_bed1_detected_) {
        barcode_bed1_pub_.publish(barcode_msg);
        barcode_bed1_detected_ = true;
        barcode_scan_start_time_ = ros::Time::now();  // 重置条码扫描计时
        ROS_INFO("[barcode_qr_detector] 条形码已发布到 /barcode_bed1: \"%s\"", data.c_str());

        // 发送到屏幕显示
        std_msgs::String display_msg;
        display_msg.data = "1床药品:" + data;
        display_text_pub_.publish(display_msg);
      } else if (!barcode_bed3_detected_) {
        barcode_bed3_pub_.publish(barcode_msg);
        barcode_bed3_detected_ = true;
        ROS_INFO("[barcode_qr_detector] 条形码已发布到 /barcode_bed3: \"%s\"", data.c_str());

        // 追加到屏幕显示
        std_msgs::String display_msg;
        display_msg.data = "1床药品:" + data + " | 3床药品:" + data;
        display_text_pub_.publish(display_msg);
      }
    }
  }

  // ── 条形码超时 ──
  if ((!barcode_bed1_detected_ || !barcode_bed3_detected_)) {
    const double bc_elapsed = (ros::Time::now() - barcode_scan_start_time_).toSec();
    if (bc_elapsed > barcode_timeout_sec_) {
      ROS_WARN("[barcode_qr_detector] 条形码检测超时 (%.1f s)", barcode_timeout_sec_);
      // 对未检测到的发布空消息
      std_msgs::String empty_msg;
      empty_msg.data = "";
      if (!barcode_bed1_detected_) {
        barcode_bed1_pub_.publish(empty_msg);
        barcode_bed1_detected_ = true;
        ROS_WARN("[barcode_qr_detector] /barcode_bed1 发布空消息（超时）");
      }
      if (!barcode_bed3_detected_) {
        barcode_bed3_pub_.publish(empty_msg);
        barcode_bed3_detected_ = true;
        ROS_WARN("[barcode_qr_detector] /barcode_bed3 发布空消息（超时）");
      }
    }
  }
}

// ── 解析 QR 码 ─────────────────────────────────────────────
bool BarcodeQrDetector::parseQrCode(const std::string& data,
                                    robot_navigation::QrResult& result) {
  // QR 内容应为恰好两位数字：如 "11", "13", "31", "33"
  if (data.length() != 2) return false;
  if (!std::isdigit(data[0]) || !std::isdigit(data[1])) return false;

  int first_bed  = data[0] - '0';   // 第一个病床号
  int first_box  = data[1] - '0';   // 第一个药箱号

  // 校验：病床号应为 1 或 3，药箱号应为 1 或 3
  if ((first_bed != 1 && first_bed != 3) ||
      (first_box != 1 && first_box != 3)) {
    return false;
  }

  // 推导第二个病床号和药箱号
  int second_bed = (first_bed == 1) ? 3 : 1;
  int second_box = (first_box == 1) ? 3 : 1;

  result.first_bed   = static_cast<int8_t>(first_bed);
  result.first_box   = static_cast<int8_t>(first_box);
  result.second_bed  = static_cast<int8_t>(second_bed);
  result.second_box  = static_cast<int8_t>(second_box);

  return true;
}

void BarcodeQrDetector::publishEmptyQrResult() {
  robot_navigation::QrResult result;
  result.first_bed   = 0;
  result.first_box   = 0;
  result.second_bed  = 0;
  result.second_box  = 0;
  qr_result_pub_.publish(result);
}

}  // namespace robot_navigation

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "barcode_qr_detector_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::BarcodeQrDetector detector(nh, pnh);
  if (!detector.init()) {
    ROS_FATAL("[barcode_qr_detector] 初始化失败");
    return 1;
  }

  ros::spin();
  return 0;
}
