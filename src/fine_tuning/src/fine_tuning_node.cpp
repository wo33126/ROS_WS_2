#include "fine_tuning/fine_tuning_controller.h"

#include <cmath>
#include <sstream>

namespace fine_tuning {

// ── 构造 ───────────────────────────────────────────────────
FineTuningController::FineTuningController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , state_(State::IDLE)
    , current_distance_mm_(0.0)
    , has_distance_(false)
    , target_distance_mm_(200.0)
    , tolerance_mm_(10.0)
    , move_axis_(1.0)          // 默认 X 轴
    , move_direction_(1)       // 默认正向
    , step_velocity_(0.05)
    , step_duration_(0.2)
    , settle_time_(0.3)
    , control_rate_hz_(10.0)
    , max_fine_tuning_time_(30.0)
    , kp_distance_(0.0005)     // 将 mm 误差映射到 m/s: 100mm 误差 → 0.05 m/s
    , max_steps_(50)
    , current_step_(0)
    , is_moving_(false)
    , is_settling_(false)
    , path_finished_received_(false)
{}

// ── 初始化 ─────────────────────────────────────────────────
bool FineTuningController::init() {
  // ── 参数读取 ──
  pnh_.param<double>("target_distance_mm", target_distance_mm_, 200.0);
  pnh_.param<double>("tolerance_mm", tolerance_mm_, 10.0);
  pnh_.param<double>("move_axis", move_axis_, 1.0);          // 1=X轴, 2=Y轴
  pnh_.param<int>("move_direction", move_direction_, 1);      // +1 或 -1
  pnh_.param<double>("step_velocity", step_velocity_, 0.05);
  pnh_.param<double>("step_duration", step_duration_, 0.2);
  pnh_.param<double>("settle_time", settle_time_, 0.3);
  pnh_.param<double>("control_rate_hz", control_rate_hz_, 10.0);
  pnh_.param<double>("max_fine_tuning_time", max_fine_tuning_time_, 30.0);
  pnh_.param<double>("kp_distance", kp_distance_, 0.0005);
  pnh_.param<int>("max_steps", max_steps_, 50);

  // 也支持从全局参数读取（与 robot_params.yaml 一致）
  nh_.param("/robot/max_linear_vel", step_velocity_,
            std::min(step_velocity_, 0.3));  // 微调速度不超过全局上限

  // ── 订阅与发布 ──
  path_finished_sub_ = nh_.subscribe("/path_finished", 1,
                                     &FineTuningController::pathFinishedCallback, this);
  distance_sub_ = nh_.subscribe("/vl53l1x_distance", 10,
                                &FineTuningController::distanceCallback, this);

  // 发布到 cmd_vel_mux 的 external 通道（path_tracker 在完成后不再发布，不会冲突）
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_external", 10);

  // 微调完成信号（latched，便于其他节点一次性读取）
  fine_tuning_done_pub_ = nh_.advertise<std_msgs::Bool>("/fine_tuning_done", 10, true);

  // ── 控制定时器 ──
  const double period = 1.0 / std::max(1.0, control_rate_hz_);
  control_timer_ = nh_.createTimer(ros::Duration(period),
                                   &FineTuningController::controlTimerCallback, this);

  // ── 服务: 外部触发微调 ──
  fine_tuning_start_srv_ = nh_.advertiseService(
      "/fine_tuning/start", &FineTuningController::fineTuningStartCallback, this);

  // ── 发布初始状态 ──
  std_msgs::Bool init_msg;
  init_msg.data = false;
  fine_tuning_done_pub_.publish(init_msg);

  ROS_INFO("[fine_tuning] 初始化完成");
  ROS_INFO("[fine_tuning]   目标距离: %.0f mm, 容差: ±%.0f mm", target_distance_mm_, tolerance_mm_);
  ROS_INFO("[fine_tuning]   移动轴: %s, 方向: %s",
           (move_axis_ == 1.0) ? "X" : "Y",
           (move_direction_ > 0) ? "正向" : "反向");
  ROS_INFO("[fine_tuning]   步进速度: %.3f m/s, 步进时长: %.2f s, 稳定等待: %.2f s",
           step_velocity_, step_duration_, settle_time_);
  ROS_INFO("[fine_tuning]   控制频率: %.1f Hz, 超时: %.0f s", control_rate_hz_, max_fine_tuning_time_);
  ROS_INFO("[fine_tuning]   等待 /path_finished 信号...");

  return true;
}

// ── /path_finished 回调 ────────────────────────────────────
void FineTuningController::pathFinishedCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data && !path_finished_received_) {
    path_finished_received_ = true;

    if (state_ == State::IDLE && has_distance_) {
      ROS_INFO("[fine_tuning] ====== 收到路径完成信号，开始位置微调 ======");
      state_ = State::FINE_TUNING;
      tuning_start_time_ = ros::Time::now();
      is_moving_ = false;
      is_settling_ = true;  // 先进入稳定等待，读取一次稳定距离
      settle_start_time_ = ros::Time::now();
      publishStop();
    } else if (!has_distance_) {
      ROS_WARN("[fine_tuning] 收到路径完成信号，但尚无 VL53L1X 距离数据，等待中...");
      // 暂存标志，等有距离数据后自动进入
    }
  }
}

// ── /vl53l1x_distance 回调 ─────────────────────────────────
void FineTuningController::distanceCallback(const std_msgs::Float32::ConstPtr& msg) {
  current_distance_mm_ = static_cast<double>(msg->data);
  has_distance_ = true;

  // 如果之前因缺少距离数据而未进入微调，现在触发
  if (path_finished_received_ && state_ == State::IDLE && has_distance_) {
    ROS_INFO("[fine_tuning] ====== 距离数据就绪，开始位置微调 ======");
    state_ = State::FINE_TUNING;
    tuning_start_time_ = ros::Time::now();
    is_moving_ = false;
    is_settling_ = true;
    settle_start_time_ = ros::Time::now();
    publishStop();
  }
}

// ── 控制定时器 ─────────────────────────────────────────────
void FineTuningController::controlTimerCallback(const ros::TimerEvent& /*event*/) {
  if (state_ != State::FINE_TUNING) {
    return;
  }

  // ── 超时保护 ──
  const double elapsed = (ros::Time::now() - tuning_start_time_).toSec();
  if (elapsed > max_fine_tuning_time_) {
    ROS_WARN("[fine_tuning] 微调超时 (%.1f s > %.1f s)，强制结束",
             elapsed, max_fine_tuning_time_);
    publishStop();
    state_ = State::DONE;

    std_msgs::Bool done_msg;
    done_msg.data = true;
    fine_tuning_done_pub_.publish(done_msg);
    ROS_INFO("[fine_tuning] ====== 微调结束（超时）======");
    return;
  }

  // ── 最大步数保护 ──
  if (max_steps_ > 0 && current_step_ >= max_steps_) {
    ROS_WARN("[fine_tuning] 微调达到最大步数 %d，强制结束", max_steps_);
    publishStop();
    state_ = State::DONE;

    std_msgs::Bool done_msg;
    done_msg.data = true;
    fine_tuning_done_pub_.publish(done_msg);
    ROS_INFO("[fine_tuning] ====== 微调结束（达到最大步数）======");
    return;
  }

  // ── 正在步进移动中 ──
  if (is_moving_) {
    const double move_elapsed = (ros::Time::now() - step_start_time_).toSec();
    if (move_elapsed >= step_duration_) {
      // 步进结束，停止移动，进入稳定等待
      publishStop();
      is_moving_ = false;
      is_settling_ = true;
      settle_start_time_ = ros::Time::now();
      ROS_DEBUG("[fine_tuning] 步进结束，等待传感器稳定 %.2f s", settle_time_);
    } else {
      // 继续发布移动指令
      double vx = 0.0, vy = 0.0;
      if (move_axis_ == 1.0) {
        vx = move_direction_ * step_velocity_;
      } else {
        vy = move_direction_ * step_velocity_;
      }
      publishVelocity(vx, vy);
    }
    return;
  }

  // ── 正在等待传感器稳定 ──
  if (is_settling_) {
    const double settle_elapsed = (ros::Time::now() - settle_start_time_).toSec();
    if (settle_elapsed < settle_time_) {
      // 仍在等待
      publishStop();
      return;
    }

    // 稳定时间到，读取距离并判断
    if (!has_distance_) {
      ROS_WARN_THROTTLE(2.0, "[fine_tuning] 等待 VL53L1X 距离数据...");
      publishStop();
      return;
    }

    const double error_mm = current_distance_mm_ - target_distance_mm_;
    const double abs_error = std::abs(error_mm);

    ROS_INFO("[fine_tuning] 当前距离: %.0f mm, 目标: %.0f mm, 误差: %+.0f mm",
             current_distance_mm_, target_distance_mm_, error_mm);

    if (abs_error <= tolerance_mm_) {
      // ── 微调完成 ──
      publishStop();
      state_ = State::DONE;

      std_msgs::Bool done_msg;
      done_msg.data = true;
      fine_tuning_done_pub_.publish(done_msg);

      ROS_INFO("[fine_tuning] ====== 微调完成！最终距离: %.0f mm, 误差: %+.0f mm ======",
               current_distance_mm_, error_mm);
      return;
    }

    // ── 需要继续微调：计算下一次步进 ──
    // 使用 PID 风格：根据误差方向决定移动方向
    const int direction = (error_mm > 0) ? move_direction_ : -move_direction_;
    // 根据误差大小调整速度（带限幅）
    double velocity = kp_distance_ * abs_error;
    velocity = clamp(velocity, 0.01, step_velocity_);  // 最低 0.01 m/s，避免太慢

    double vx = 0.0, vy = 0.0;
    if (move_axis_ == 1.0) {
      vx = direction * velocity;
    } else {
      vy = direction * velocity;
    }

    ROS_INFO("[fine_tuning] 执行步进: vx=%.3f, vy=%.3f, 持续 %.2f s",
             vx, vy, step_duration_);

    publishVelocity(vx, vy);
    is_moving_ = true;
    is_settling_ = false;
    step_start_time_ = ros::Time::now();
    current_step_++;
  }
}

// ── /fine_tuning/start 服务回调（非阻塞） ────────────────────
bool FineTuningController::fineTuningStartCallback(
    std_srvs::Trigger::Request& /*req*/,
    std_srvs::Trigger::Response& res) {
  if (state_ == State::FINE_TUNING) {
    res.success = false;
    res.message = "微调已在进行中";
    ROS_WARN("[fine_tuning] /fine_tuning/start 被拒绝: %s", res.message.c_str());
    return true;
  }

  if (!has_distance_) {
    res.success = false;
    res.message = "尚无 VL53L1X 距离数据，无法微调";
    ROS_WARN("[fine_tuning] /fine_tuning/start 被拒绝: %s", res.message.c_str());
    return true;
  }

  ROS_INFO("[fine_tuning] ====== 收到 /fine_tuning/start 服务请求，开始微调 ======");
  state_ = State::FINE_TUNING;
  tuning_start_time_ = ros::Time::now();
  current_step_ = 0;
  is_moving_ = false;
  is_settling_ = true;
  settle_start_time_ = ros::Time::now();
  publishStop();

  // 清空上次的完成信号
  {
    std_msgs::Bool clear_msg;
    clear_msg.data = false;
    fine_tuning_done_pub_.publish(clear_msg);
  }

  // 非阻塞：立即返回，微调由 controlTimerCallback 驱动
  // mission_controller 通过 /fine_tuning_done 话题获知完成
  res.success = true;
  res.message = "微调已启动（异步执行）";
  ROS_INFO("[fine_tuning] /fine_tuning/start 服务: 微调已启动");

  return true;
}

// ── 停止 ───────────────────────────────────────────────────
void FineTuningController::publishStop() {
  geometry_msgs::Twist stop;
  cmd_vel_pub_.publish(stop);
}

// ── 发布速度 ───────────────────────────────────────────────
void FineTuningController::publishVelocity(double vx, double vy) {
  geometry_msgs::Twist twist;
  twist.linear.x = vx;
  twist.linear.y = vy;
  twist.angular.z = 0.0;
  cmd_vel_pub_.publish(twist);
}

}  // namespace fine_tuning

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "fine_tuning_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  fine_tuning::FineTuningController controller(nh, pnh);
  if (!controller.init()) {
    ROS_FATAL("[fine_tuning] 初始化失败");
    return 1;
  }

  ros::spin();
  return 0;
}
