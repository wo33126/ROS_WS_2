#pragma once

#include <ros/ros.h>

#include <robot_navigation/QrResult.h>
#include <robot_navigation/OpenMedicineBox.h>
#include <robot_navigation/ArmPlaceMedicine.h>
#include <robot_navigation/Speak.h>

#include <path_manager/SelectPath.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

namespace robot_navigation {

/**
 * @brief 任务状态机节点
 *
 * 管理比赛全流程状态机：
 *   IDLE → GOTO_NURSE → SCAN_QR → GOTO_BED_A → POSITION_IN_CIRCLE →
 *   SCAN_BARCODE → OPEN_BOX → PLACE_MEDICINE → VOICE →
 *   GOTO_BED_B → ... → RETURN_HOME → STOP
 *
 * 全局 180 秒超时保护，各阶段 30 秒超时保护。
 */
class MissionController {
 public:
  MissionController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~MissionController() = default;

  bool init();

 private:
  // ── 状态机枚举 ──
  enum class State {
    IDLE = 0,
    GOTO_NURSE,
    SCAN_QR,
    GOTO_BED_A,
    POSITION_IN_CIRCLE_A,
    SCAN_BARCODE_A,
    OPEN_BOX_A,
    PLACE_MEDICINE_A,
    VOICE_A,
    GOTO_BED_B,
    POSITION_IN_CIRCLE_B,
    SCAN_BARCODE_B,
    OPEN_BOX_B,
    PLACE_MEDICINE_B,
    VOICE_B,
    RETURN_HOME,
    HOME_CHECK,
    STOP,
    TIMEOUT,
    FAILED
  };

  // ── 回调 ──
  void qrResultCallback(const robot_navigation::QrResult::ConstPtr& msg);
  void barcodeBed1Callback(const std_msgs::String::ConstPtr& msg);
  void barcodeBed3Callback(const std_msgs::String::ConstPtr& msg);
  void startSignalCallback(const std_msgs::Empty::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void missionTimerCallback(const ros::TimerEvent& event);
  void stateMachineTimerCallback(const ros::TimerEvent& event);

  // ── 状态处理 ──
  void processState();
  bool executeStateTransition();

  // ── 服务调用辅助 ──
  bool callSelectPath(const std::string& path_name);
  bool callFineTuningStart();
  bool callOpenMedicineBox(int8_t box_id);
  bool callArmPlaceMedicine(int8_t bed_id);
  bool callSpeak(const std::string& text);

  // ── 回归检测 ──
  bool isRobotInHomeZone() const;

  // ── 状态描述 ──
  std::string stateToString(State s) const;

  // ── 工具 ──
  double clamp(double val, double lo, double hi) const {
    return std::max(lo, std::min(val, hi));
  }

  // ── ROS 接口 ──
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // 订阅
  ros::Subscriber qr_result_sub_;
  ros::Subscriber barcode_bed1_sub_;
  ros::Subscriber barcode_bed3_sub_;
  ros::Subscriber start_signal_sub_;
  ros::Subscriber odom_sub_;

  // 发布
  ros::Publisher mission_finished_pub_;
  ros::Publisher mission_timeout_pub_;
  ros::Publisher stop_all_pub_;
  ros::Publisher display_text_pub_;

  // 服务客户端
  ros::ServiceClient path_select_client_;
  ros::ServiceClient fine_tuning_client_;     // std_srvs::Trigger
  ros::ServiceClient open_box_client_;
  ros::ServiceClient arm_place_client_;
  ros::ServiceClient speak_client_;

  // 定时器
  ros::Timer mission_timer_;      // 全局 180s 超时
  ros::Timer state_machine_timer_; // 状态机主循环

  // ── 状态机变量 ──
  State current_state_;
  std::atomic<bool> mission_started_;
  std::atomic<bool> mission_completed_;

  // ── QR 结果 ──
  QrResult qr_result_;
  bool qr_received_;

  // ── 条形码 ──
  std::string barcode_bed1_value_;
  std::string barcode_bed3_value_;
  bool barcode_bed1_received_;
  bool barcode_bed3_received_;

  // ── 里程计 ──
  nav_msgs::Odometry current_odom_;
  bool odom_received_;
  mutable std::mutex odom_mutex_;

  // ── 阶段计时 ──
  ros::Time stage_start_time_;

  // ── 回归检测 ──
  ros::Time home_arrival_time_;
  bool home_arrived_;

  // ── 参数 ──
  double stage_timeout_sec_;       // 单阶段超时 (默认 30s)
  double mission_timeout_sec_;     // 全局超时 (默认 180s)
  double state_machine_rate_hz_;   // 状态机循环频率

  // ── 路径名称 ──
  std::string path_nurse_station_;
  std::string path_bed1_circle_;
  std::string path_bed3_circle_;
  std::string path_home_;

  // ── 圆圈参数（用于微调目标） ──
  struct CircleDef {
    double center_x;
    double center_y;
    double radius;
  };
  CircleDef circle_bed1_;
  CircleDef circle_bed3_;

  // ── 起始区多边形顶点 ──
  struct Point2D {
    double x;
    double y;
  };
  std::vector<Point2D> home_zone_vertices_;

  // ── 回归稳定时间 ──
  double home_hold_sec_;
};

}  // namespace robot_navigation
