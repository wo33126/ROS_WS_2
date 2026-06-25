#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

class TwistTestNode {
 public:
  TwistTestNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh), vx_(0.5), vy_(0.0), wz_(0.0), rate_hz_(10.0) {
    pnh_.param("vx", vx_, vx_);
    pnh_.param("vy", vy_, vy_);
    pnh_.param("wz", wz_, wz_);
    pnh_.param("rate_hz", rate_hz_, rate_hz_);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    rpm_sub_ = nh_.subscribe("/motor_velocity_cmd", 10, &TwistTestNode::rpmCallback, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, rate_hz_)), &TwistTestNode::timerCallback, this);
  }

 private:
  void timerCallback(const ros::TimerEvent&) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = vx_;
    cmd.linear.y = vy_;
    cmd.angular.z = wz_;
    cmd_pub_.publish(cmd);
  }

  void rpmCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 4) {
      return;
    }
    ROS_INFO_THROTTLE(0.5,
                      "twist_test: cmd(vx=%.2f, vy=%.2f, wz=%.2f) -> rpm[F,R,B,L]=[%.1f, %.1f, %.1f, %.1f]",
                      vx_,
                      vy_,
                      wz_,
                      msg->data[0],
                      msg->data[1],
                      msg->data[2],
                      msg->data[3]);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher cmd_pub_;
  ros::Subscriber rpm_sub_;
  ros::Timer timer_;

  double vx_;
  double vy_;
  double wz_;
  double rate_hz_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "twist_test_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  TwistTestNode node(nh, pnh);
  ros::spin();
  return 0;
}
