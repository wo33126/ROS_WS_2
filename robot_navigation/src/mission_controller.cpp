/**
 * @file mission_controller.cpp
 * @brief 任务状态机节点实现
 *
 * 管理比赛全流程：
 *   IDLE → GOTO_NURSE → SCAN_QR → GOTO_BED_A → POSITION_IN_CIRCLE →
 *   SCAN_BARCODE → OPEN_BOX → PLACE_MEDICINE → VOICE →
 *   GOTO_BED_B → ... → RETURN_HOME → STOP
 */

#include "robot_navigation/mission_controller.h"

namespace robot_navigation {

// ── 构造 ───────────────────────────────────────────────────
MissionController::MissionController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , current_state_(State::IDLE)
    , mission_started_(false)
    , mission_completed_(false)
    , qr_received_(false)
    , barcode_bed1_received_(false)
    , barcode_bed3_received_(false)
    , odom_received_(false)
    , home_arrived_(false)
    , stage_timeout_sec_(30.0)
    , mission_timeout_sec_(180.0)
    , state_machine_rate_hz_(10.0)
    , path_nurse_station_("nurse_station")
    , path_bed1_circle_("bed1_circle")
    , path_bed3_circle_("bed3_circle")
    , path_home_("HOME")
    , home_hold_sec_(5.0)
{
  // ── 参数读取 ──
  pnh_.param<double>("stage_timeout_sec", stage_timeout_sec_, 30.0);
  pnh_.param<double>("mission_timeout_sec", mission_timeout_sec_, 180.0);
  pnh_.param<double>("state_machine_rate_hz", state_machine_rate_hz_, 10.0);

  pnh_.param<std::string>("path_nurse_station", path_nurse_station_, "nurse_station");
  pnh_.param<std::string>("path_bed1_circle", path_bed1_circle_, "bed1_circle");
  pnh_.param<std::string>("path_bed3_circle", path_bed3_circle_, "bed3_circle");
  pnh_.param<std::string>("path_home", path_home_, "HOME");

  // 圆圈参数
  pnh_.param<double>("circle_bed1_x", circle_bed1_.center_x, 0.0);
  pnh_.param<double>("circle_bed1_y", circle_bed1_.center_y, 0.0);
  pnh_.param<double>("circle_bed1_radius", circle_bed1_.radius, 0.3);

  pnh_.param<double>("circle_bed3_x", circle_bed3_.center_x, 0.0);
  pnh_.param<double>("circle_bed3_y", circle_bed3_.center_y, 0.0);
  pnh_.param<double>("circle_bed3_radius", circle_bed3_.radius, 0.3);

  pnh_.param<double>("home_hold_sec", home_hold_sec_, 5.0);

  // 起始区多边形顶点（从参数加载）
  XmlRpc::XmlRpcValue home_vertices;
  if (pnh_.getParam("home_zone_vertices", home_vertices) &&
      home_vertices.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < home_vertices.size(); ++i) {
      Point2D pt;
      pt.x = static_cast<double>(home_vertices[i]["x"]);
      pt.y = static_cast<double>(home_vertices[i]["y"]);
      home_zone_vertices_.push_back(pt);
    }
  }
}

bool MissionController::init() {
  // ── 订阅 ──
  qr_result_sub_     = nh_.subscribe("/qr_result", 1,
                                     &MissionController::qrResultCallback, this);
  barcode_bed1_sub_  = nh_.subscribe("/barcode_bed1", 1,
                                     &MissionController::barcodeBed1Callback, this);
  barcode_bed3_sub_  = nh_.subscribe("/barcode_bed3", 1,
                                     &MissionController::barcodeBed3Callback, this);
  start_signal_sub_  = nh_.subscribe("/start_signal", 1,
                                     &MissionController::startSignalCallback, this);
  odom_sub_          = nh_.subscribe("/odom", 10,
                                     &MissionController::odomCallback, this);

  // ── 发布 ──
  mission_finished_pub_ = nh_.advertise<std_msgs::Bool>("/mission_finished", 1, true);
  mission_timeout_pub_  = nh_.advertise<std_msgs::Bool>("/mission_timeout", 1, true);
  stop_all_pub_         = nh_.advertise<std_msgs::Empty>("/stop_all", 1, true);
  display_text_pub_     = nh_.advertise<std_msgs::String>("/robot_display", 1, true);

  // ── 服务客户端 ──
  ROS_INFO("[mission_controller] 等待服务就绪...");
  const ros::Duration service_timeout(10.0);

  if (!ros::service::waitForService("/select_path", service_timeout)) {
    ROS_WARN("[mission_controller] /select_path 未就绪");
  }
  path_select_client_ = nh_.serviceClient<path_manager::SelectPath>("/select_path");

  if (!ros::service::waitForService("/fine_tuning/start", service_timeout)) {
    ROS_WARN("[mission_controller] /fine_tuning/start 未就绪");
  }
  fine_tuning_client_ = nh_.serviceClient<std_srvs::Trigger>("/fine_tuning/start");

  if (!ros::service::waitForService("/open_medicine_box", service_timeout)) {
    ROS_WARN("[mission_controller] /open_medicine_box 未就绪");
  }
  open_box_client_ = nh_.serviceClient<robot_navigation::OpenMedicineBox>("/open_medicine_box");

  if (!ros::service::waitForService("/arm_place_medicine", service_timeout)) {
    ROS_WARN("[mission_controller] /arm_place_medicine 未就绪");
  }
  arm_place_client_ = nh_.serviceClient<robot_navigation::ArmPlaceMedicine>("/arm_place_medicine");

  if (!ros::service::waitForService("/speak", service_timeout)) {
    ROS_WARN("[mission_controller] /speak 未就绪");
  }
  speak_client_ = nh_.serviceClient<robot_navigation::Speak>("/speak");

  // ── 定时器 ──
  // 全局任务定时器（3分钟），先不启动，收到 start_signal 后再启动
  mission_timer_ = nh_.createTimer(ros::Duration(mission_timeout_sec_),
                                   &MissionController::missionTimerCallback,
                                   this, true, false);  // oneshot, 初始不启动

  // 状态机主循环
  const double period = 1.0 / std::max(1.0, state_machine_rate_hz_);
  state_machine_timer_ = nh_.createTimer(ros::Duration(period),
                                         &MissionController::stateMachineTimerCallback,
                                         this);

  // ── 初始显示 ──
  {
    std_msgs::String init_display;
    init_display.data = "等待启动信号...";
    display_text_pub_.publish(init_display);
  }

  ROS_INFO("[mission_controller] 初始化完成，当前状态: %s",
           stateToString(current_state_).c_str());
  ROS_INFO("[mission_controller]   阶段超时: %.0f s, 全局超时: %.0f s",
           stage_timeout_sec_, mission_timeout_sec_);
  ROS_INFO("[mission_controller]   等待 /start_signal 信号...");

  return true;
}

// ── 回调: QR 结果 ──────────────────────────────────────────
void MissionController::qrResultCallback(const robot_navigation::QrResult::ConstPtr& msg) {
  if (qr_received_) return;

  qr_result_ = *msg;
  qr_received_ = true;

  ROS_INFO("[mission_controller] 收到 QR 结果: first_bed=%d, first_box=%d, "
           "second_bed=%d, second_box=%d",
           msg->first_bed, msg->first_box,
           msg->second_bed, msg->second_box);
}

// ── 回调: 条形码 bed1 ──────────────────────────────────────
void MissionController::barcodeBed1Callback(const std_msgs::String::ConstPtr& msg) {
  if (barcode_bed1_received_) return;
  barcode_bed1_value_ = msg->data;
  barcode_bed1_received_ = true;
  ROS_INFO("[mission_controller] 收到 1 床条形码: \"%s\"", msg->data.c_str());

  // 更新屏幕显示
  std_msgs::String display;
  display.data = "1床药品:" + barcode_bed1_value_ +
                 (barcode_bed3_received_ ? (" | 3床药品:" + barcode_bed3_value_) : "");
  display_text_pub_.publish(display);
}

// ── 回调: 条形码 bed3 ──────────────────────────────────────
void MissionController::barcodeBed3Callback(const std_msgs::String::ConstPtr& msg) {
  if (barcode_bed3_received_) return;
  barcode_bed3_value_ = msg->data;
  barcode_bed3_received_ = true;
  ROS_INFO("[mission_controller] 收到 3 床条形码: \"%s\"", msg->data.c_str());

  std_msgs::String display;
  display.data = (barcode_bed1_received_ ? ("1床药品:" + barcode_bed1_value_ + " | ") : "") +
                 "3床药品:" + barcode_bed3_value_;
  display_text_pub_.publish(display);
}

// ── 回调: 启动信号 ─────────────────────────────────────────
void MissionController::startSignalCallback(const std_msgs::Empty::ConstPtr& /*msg*/) {
  if (mission_started_.load()) return;

  mission_started_.store(true);
  current_state_ = State::GOTO_NURSE;
  stage_start_time_ = ros::Time::now();

  // 启动全局超时定时器
  mission_timer_.start();

  ROS_INFO("[mission_controller] ====== 收到启动信号！开始任务 ======");

  std_msgs::String display;
  display.data = "任务开始";
  display_text_pub_.publish(display);
}

// ── 回调: 里程计 ───────────────────────────────────────────
void MissionController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  current_odom_ = *msg;
  odom_received_ = true;
}

// ── 全局超时回调 ───────────────────────────────────────────
void MissionController::missionTimerCallback(const ros::TimerEvent& /*event*/) {
  ROS_ERROR("[mission_controller] ====== 全局任务超时！(%.0f s) ======",
            mission_timeout_sec_);
  current_state_ = State::TIMEOUT;

  std_msgs::Bool timeout_msg;
  timeout_msg.data = true;
  mission_timeout_pub_.publish(timeout_msg);

  // 停止所有运动
  std_msgs::Empty stop;
  stop_all_pub_.publish(stop);

  std_msgs::String display;
  display.data = "任务超时!";
  display_text_pub_.publish(display);
}

// ── 状态机主循环 ───────────────────────────────────────────
void MissionController::stateMachineTimerCallback(const ros::TimerEvent& /*event*/) {
  if (!mission_started_.load() || mission_completed_.load()) return;

  processState();
}

// ── 状态处理 ───────────────────────────────────────────────
void MissionController::processState() {
  if (current_state_ == State::IDLE ||
      current_state_ == State::STOP ||
      current_state_ == State::TIMEOUT ||
      current_state_ == State::FAILED) {
    return;
  }

  bool success = executeStateTransition();
  if (!success) {
    ROS_ERROR("[mission_controller] 状态 %s 执行失败，进入失败状态",
              stateToString(current_state_).c_str());
    current_state_ = State::FAILED;

    std_msgs::Bool fail_msg;
    fail_msg.data = false;
    mission_finished_pub_.publish(fail_msg);

    std_msgs::String display;
    display.data = "任务失败: " + stateToString(current_state_);
    display_text_pub_.publish(display);
  }
}

// ── 状态迁移 ───────────────────────────────────────────────
bool MissionController::executeStateTransition() {
  switch (current_state_) {
    // ── GOTO_NURSE: 导航到护士台 ──
    case State::GOTO_NURSE: {
      ROS_INFO("[mission_controller] >>> GOTO_NURSE: 导航到护士台");
      if (!callSelectPath(path_nurse_station_)) {
        ROS_ERROR("[mission_controller] 选择路径 '%s' 失败", path_nurse_station_.c_str());
        return false;
      }
      // 等待路径跟踪完成（由 path_tracker 发布 /path_finished 信号）
      // 这里简单延时等待，实际应订阅 /path_finished 话题
      // 简化实现：等待 stage_timeout 后进入下一状态
      ROS_INFO("[mission_controller] 等待路径跟踪完成 (最多 %.0f s)...", stage_timeout_sec_);
      ros::Duration wait_dur = ros::Duration(std::min(stage_timeout_sec_, 15.0));
      wait_dur.sleep();
      current_state_ = State::SCAN_QR;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── SCAN_QR: 扫描护士台二维码 ──
    case State::SCAN_QR: {
      ROS_INFO("[mission_controller] >>> SCAN_QR: 等待二维码识别");
      // 等待 QR 结果
      ros::Rate r(10);
      ros::Time start = ros::Time::now();
      while (!qr_received_ && ros::ok()) {
        ros::spinOnce();
        r.sleep();
        if ((ros::Time::now() - start).toSec() > stage_timeout_sec_) {
          ROS_ERROR("[mission_controller] QR 扫描超时");
          return false;
        }
      }
      if (qr_result_.first_bed == 0 && qr_result_.first_box == 0) {
        ROS_ERROR("[mission_controller] QR 结果为空");
        return false;
      }
      current_state_ = State::GOTO_BED_A;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── GOTO_BED_A: 导航到第一个病床圆圈附近 ──
    case State::GOTO_BED_A: {
      int bed = qr_result_.first_bed;
      std::string path = (bed == 1) ? path_bed1_circle_ : path_bed3_circle_;
      ROS_INFO("[mission_controller] >>> GOTO_BED_A: 导航到 %d 床 (%s)", bed, path.c_str());
      if (!callSelectPath(path)) return false;
      ros::Duration(std::min(stage_timeout_sec_, 15.0)).sleep();
      current_state_ = State::POSITION_IN_CIRCLE_A;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── POSITION_IN_CIRCLE_A: 微调进入圆圈 ──
    case State::POSITION_IN_CIRCLE_A: {
      ROS_INFO("[mission_controller] >>> POSITION_IN_CIRCLE_A: 启动微调");
      if (!callFineTuningStart()) {
        ROS_ERROR("[mission_controller] 微调失败");
        return false;
      }
      current_state_ = State::SCAN_BARCODE_A;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── SCAN_BARCODE_A: 扫描药品条形码 ──
    case State::SCAN_BARCODE_A: {
      int bed = qr_result_.first_bed;
      ROS_INFO("[mission_controller] >>> SCAN_BARCODE_A: 等待 %d 床条形码", bed);

      // 等待对应病床的条形码
      ros::Rate r(10);
      ros::Time start = ros::Time::now();
      bool& received = (bed == 1) ? barcode_bed1_received_ : barcode_bed3_received_;
      while (!received && ros::ok()) {
        ros::spinOnce();
        r.sleep();
        if ((ros::Time::now() - start).toSec() > stage_timeout_sec_) {
          ROS_ERROR("[mission_controller] 条形码扫描超时");
          return false;
        }
      }
      current_state_ = State::OPEN_BOX_A;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── OPEN_BOX_A: 打开药箱 ──
    case State::OPEN_BOX_A: {
      int box = qr_result_.first_box;
      ROS_INFO("[mission_controller] >>> OPEN_BOX_A: 打开药箱 %d", box);
      if (!callOpenMedicineBox(box)) {
        ROS_ERROR("[mission_controller] 打开药箱 %d 失败", box);
        return false;
      }
      current_state_ = State::PLACE_MEDICINE_A;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── PLACE_MEDICINE_A: 放置药品 ──
    case State::PLACE_MEDICINE_A: {
      int bed = qr_result_.first_bed;
      ROS_INFO("[mission_controller] >>> PLACE_MEDICINE_A: 放置药品到 %d 床", bed);
      if (!callArmPlaceMedicine(bed)) {
        ROS_ERROR("[mission_controller] 放置药品到 %d 床失败", bed);
        return false;
      }
      current_state_ = State::VOICE_A;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── VOICE_A: 语音播报 ──
    case State::VOICE_A: {
      int bed = qr_result_.first_bed;
      std::string text = std::to_string(bed) + "床病人请取药";
      ROS_INFO("[mission_controller] >>> VOICE_A: 播报 \"%s\"", text.c_str());
      if (!callSpeak(text)) {
        ROS_WARN("[mission_controller] 语音播报失败，继续执行");
      }
      current_state_ = State::GOTO_BED_B;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── GOTO_BED_B: 导航到第二个病床 ──
    case State::GOTO_BED_B: {
      int bed = qr_result_.second_bed;
      std::string path = (bed == 1) ? path_bed1_circle_ : path_bed3_circle_;
      ROS_INFO("[mission_controller] >>> GOTO_BED_B: 导航到 %d 床 (%s)", bed, path.c_str());
      if (!callSelectPath(path)) return false;
      ros::Duration(std::min(stage_timeout_sec_, 15.0)).sleep();
      current_state_ = State::POSITION_IN_CIRCLE_B;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── POSITION_IN_CIRCLE_B ──
    case State::POSITION_IN_CIRCLE_B: {
      ROS_INFO("[mission_controller] >>> POSITION_IN_CIRCLE_B: 启动微调");
      if (!callFineTuningStart()) return false;
      current_state_ = State::SCAN_BARCODE_B;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── SCAN_BARCODE_B ──
    case State::SCAN_BARCODE_B: {
      int bed = qr_result_.second_bed;
      ROS_INFO("[mission_controller] >>> SCAN_BARCODE_B: 等待 %d 床条形码", bed);

      ros::Rate r(10);
      ros::Time start = ros::Time::now();
      bool& received = (bed == 1) ? barcode_bed1_received_ : barcode_bed3_received_;
      while (!received && ros::ok()) {
        ros::spinOnce();
        r.sleep();
        if ((ros::Time::now() - start).toSec() > stage_timeout_sec_) return false;
      }
      current_state_ = State::OPEN_BOX_B;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── OPEN_BOX_B ──
    case State::OPEN_BOX_B: {
      int box = qr_result_.second_box;
      ROS_INFO("[mission_controller] >>> OPEN_BOX_B: 打开药箱 %d", box);
      if (!callOpenMedicineBox(box)) return false;
      current_state_ = State::PLACE_MEDICINE_B;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── PLACE_MEDICINE_B ──
    case State::PLACE_MEDICINE_B: {
      int bed = qr_result_.second_bed;
      ROS_INFO("[mission_controller] >>> PLACE_MEDICINE_B: 放置药品到 %d 床", bed);
      if (!callArmPlaceMedicine(bed)) return false;
      current_state_ = State::VOICE_B;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── VOICE_B ──
    case State::VOICE_B: {
      int bed = qr_result_.second_bed;
      std::string text = std::to_string(bed) + "床病人请取药";
      ROS_INFO("[mission_controller] >>> VOICE_B: 播报 \"%s\"", text.c_str());
      callSpeak(text);  // 失败也不阻止
      current_state_ = State::RETURN_HOME;
      stage_start_time_ = ros::Time::now();
      return true;
    }

    // ── RETURN_HOME: 返回起始区 ──
    case State::RETURN_HOME: {
      ROS_INFO("[mission_controller] >>> RETURN_HOME: 返回起始区");
      if (!callSelectPath(path_home_)) return false;
      ros::Duration(std::min(stage_timeout_sec_, 15.0)).sleep();
      current_state_ = State::HOME_CHECK;
      stage_start_time_ = ros::Time::now();
      home_arrived_ = false;
      return true;
    }

    // ── HOME_CHECK: 检测是否在起始框内并静止 5 秒 ──
    case State::HOME_CHECK: {
      ros::Rate r(10);
      ros::Time start = ros::Time::now();

      while (ros::ok()) {
        ros::spinOnce();
        if (!isRobotInHomeZone()) {
          home_arrived_ = false;
          home_arrival_time_ = ros::Time::now();
        } else if (!home_arrived_) {
          home_arrived_ = true;
          home_arrival_time_ = ros::Time::now();
          ROS_INFO("[mission_controller] 机器人已进入起始区");
        }

        if (home_arrived_) {
          double hold = (ros::Time::now() - home_arrival_time_).toSec();
          if (hold >= home_hold_sec_) {
            ROS_INFO("[mission_controller] 机器人在起始区内稳定 %.1f 秒", hold);
            break;
          }
        }

        if ((ros::Time::now() - start).toSec() > stage_timeout_sec_) {
          ROS_ERROR("[mission_controller] 回归起始区超时");
          return false;
        }
        r.sleep();
      }

      // ── 全部完成! ──
      current_state_ = State::STOP;
      mission_completed_.store(true);

      std_msgs::Empty stop;
      stop_all_pub_.publish(stop);

      std_msgs::Bool done;
      done.data = true;
      mission_finished_pub_.publish(done);

      std_msgs::String display;
      display.data = "任务完成! 1床:" + barcode_bed1_value_ +
                     " | 3床:" + barcode_bed3_value_;
      display_text_pub_.publish(display);

      ROS_INFO("[mission_controller] ====== 全部任务完成！======");
      return true;
    }

    default:
      return true;
  }
}

// ── 服务调用辅助 ───────────────────────────────────────────
bool MissionController::callSelectPath(const std::string& path_name) {
  if (!path_select_client_.exists()) {
    ROS_ERROR("[mission_controller] /select_path 服务不可用");
    return false;
  }

  path_manager::SelectPath srv;
  srv.request.path_name = path_name;

  if (!path_select_client_.call(srv)) {
    ROS_ERROR("[mission_controller] 调用 /select_path('%s') 失败", path_name.c_str());
    return false;
  }
  if (!srv.response.success) {
    ROS_ERROR("[mission_controller] /select_path('%s') 返回失败: %s",
              path_name.c_str(), srv.response.message.c_str());
    return false;
  }

  ROS_INFO("[mission_controller] 路径 '%s' 选择成功", path_name.c_str());
  return true;
}

bool MissionController::callFineTuningStart() {
  if (!fine_tuning_client_.exists()) {
    ROS_ERROR("[mission_controller] /fine_tuning/start 服务不可用");
    return false;
  }

  std_srvs::Trigger srv;
  if (!fine_tuning_client_.call(srv)) {
    ROS_ERROR("[mission_controller] 调用 /fine_tuning/start 失败");
    return false;
  }
  if (!srv.response.success) {
    ROS_ERROR("[mission_controller] /fine_tuning/start 返回失败: %s",
              srv.response.message.c_str());
    return false;
  }

  ROS_INFO("[mission_controller] 微调完成");
  return true;
}

bool MissionController::callOpenMedicineBox(int8_t box_id) {
  if (!open_box_client_.exists()) {
    ROS_ERROR("[mission_controller] /open_medicine_box 服务不可用");
    return false;
  }

  robot_navigation::OpenMedicineBox srv;
  srv.request.box_id = box_id;

  if (!open_box_client_.call(srv)) {
    ROS_ERROR("[mission_controller] 调用 /open_medicine_box(%d) 失败", box_id);
    return false;
  }
  if (!srv.response.success) {
    ROS_ERROR("[mission_controller] /open_medicine_box(%d) 返回失败: %s",
              box_id, srv.response.message.c_str());
    return false;
  }

  ROS_INFO("[mission_controller] 药箱 %d 已打开", box_id);
  return true;
}

bool MissionController::callArmPlaceMedicine(int8_t bed_id) {
  if (!arm_place_client_.exists()) {
    ROS_ERROR("[mission_controller] /arm_place_medicine 服务不可用");
    return false;
  }

  robot_navigation::ArmPlaceMedicine srv;
  srv.request.bed_id = bed_id;

  if (!arm_place_client_.call(srv)) {
    ROS_ERROR("[mission_controller] 调用 /arm_place_medicine(%d) 失败", bed_id);
    return false;
  }
  if (!srv.response.success) {
    ROS_ERROR("[mission_controller] /arm_place_medicine(%d) 返回失败: %s",
              bed_id, srv.response.message.c_str());
    return false;
  }

  ROS_INFO("[mission_controller] 药品已放置到 %d 床", bed_id);
  return true;
}

bool MissionController::callSpeak(const std::string& text) {
  if (!speak_client_.exists()) {
    ROS_WARN("[mission_controller] /speak 服务不可用，跳过语音播报");
    return false;
  }

  robot_navigation::Speak srv;
  srv.request.text = text;

  if (!speak_client_.call(srv)) {
    ROS_WARN("[mission_controller] 调用 /speak 失败");
    return false;
  }
  ROS_INFO("[mission_controller] 语音播报: \"%s\"", text.c_str());
  return srv.response.success;
}

// ── 判断机器人是否在起始区内 ───────────────────────────────
bool MissionController::isRobotInHomeZone() const {
  if (!odom_received_ || home_zone_vertices_.empty()) {
    // 无里程计数据或无配置，回退到圆形判定
    return false;
  }

  std::lock_guard<std::mutex> lock(odom_mutex_);
  double rx = current_odom_.pose.pose.position.x;
  double ry = current_odom_.pose.pose.position.y;

  // 射线法判断点是否在多边形内
  int crossings = 0;
  const size_t n = home_zone_vertices_.size();
  for (size_t i = 0; i < n; ++i) {
    const Point2D& p1 = home_zone_vertices_[i];
    const Point2D& p2 = home_zone_vertices_[(i + 1) % n];

    if (((p1.y > ry) != (p2.y > ry)) &&
        (rx < (p2.x - p1.x) * (ry - p1.y) / (p2.y - p1.y) + p1.x)) {
      crossings++;
    }
  }

  return (crossings % 2 == 1);
}

// ── 状态描述 ───────────────────────────────────────────────
std::string MissionController::stateToString(State s) const {
  switch (s) {
    case State::IDLE:               return "IDLE";
    case State::GOTO_NURSE:         return "GOTO_NURSE";
    case State::SCAN_QR:            return "SCAN_QR";
    case State::GOTO_BED_A:         return "GOTO_BED_A";
    case State::POSITION_IN_CIRCLE_A: return "POSITION_IN_CIRCLE_A";
    case State::SCAN_BARCODE_A:     return "SCAN_BARCODE_A";
    case State::OPEN_BOX_A:         return "OPEN_BOX_A";
    case State::PLACE_MEDICINE_A:   return "PLACE_MEDICINE_A";
    case State::VOICE_A:            return "VOICE_A";
    case State::GOTO_BED_B:         return "GOTO_BED_B";
    case State::POSITION_IN_CIRCLE_B: return "POSITION_IN_CIRCLE_B";
    case State::SCAN_BARCODE_B:     return "SCAN_BARCODE_B";
    case State::OPEN_BOX_B:         return "OPEN_BOX_B";
    case State::PLACE_MEDICINE_B:   return "PLACE_MEDICINE_B";
    case State::VOICE_B:            return "VOICE_B";
    case State::RETURN_HOME:        return "RETURN_HOME";
    case State::HOME_CHECK:         return "HOME_CHECK";
    case State::STOP:               return "STOP";
    case State::TIMEOUT:            return "TIMEOUT";
    case State::FAILED:             return "FAILED";
    default:                        return "UNKNOWN";
  }
}

}  // namespace robot_navigation

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::MissionController controller(nh, pnh);
  if (!controller.init()) {
    ROS_FATAL("[mission_controller] 初始化失败");
    return 1;
  }

  ros::spin();
  return 0;
}
