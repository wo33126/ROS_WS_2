#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <path_manager/PathManagerConfig.h>

#include <path_manager/PathPoint.h>
#include <path_manager/LoadPath.h>
#include <path_manager/SelectPath.h>
#include <std_srvs/Trigger.h>

#include <sstream>
#include <cstdlib>

// ── 内部数据结构 ────────────────────────────────────────────
struct PointDef {
  double x;
  double y;
  double tolerance;
};

struct PathDef {
  std::string name;
  std::vector<PointDef> points;
};

// ── PathManager 类 ─────────────────────────────────────────
class PathManager {
 public:
  PathManager()
    : pnh_("~")
    , active_path_idx_(-1)
    , active_point_idx_(0)
    , override_enabled_(false)
    , override_x_(0.0)
    , override_y_(0.0)
    , override_tolerance_(0.05)
    , publish_rate_hz_(10.0)
  {}

  bool init();

 private:
  // ── ROS 接口 ──
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher  path_pub_;
  ros::ServiceServer load_srv_;
  ros::ServiceServer select_srv_;
  ros::ServiceServer next_srv_;
  ros::Timer        pub_timer_;

  dynamic_reconfigure::Server<path_manager::PathManagerConfig> cfg_server_;
  dynamic_reconfigure::Server<path_manager::PathManagerConfig>::CallbackType cfg_cb_;

  // ── 路径数据 ──
  std::vector<PathDef> paths_;
  int active_path_idx_;
  int active_point_idx_;

  // ── 动态参数覆盖 ──
  bool   override_enabled_;
  double override_x_;
  double override_y_;
  double override_tolerance_;

  // ── 配置 ──
  double publish_rate_hz_;
  std::string yaml_file_path_;

  // ── 内部方法 ──
  bool loadPathsFromParam();
  bool loadPathsFromFile(const std::string& file_path);
  void publishTimerCallback(const ros::TimerEvent& event);
  bool handleLoadPath(path_manager::LoadPath::Request&  req,
                      path_manager::LoadPath::Response& res);
  bool handleSelectPath(path_manager::SelectPath::Request&  req,
                        path_manager::SelectPath::Response& res);
  bool handleNextPoint(std_srvs::Trigger::Request&  req,
                       std_srvs::Trigger::Response& res);
  void reconfigureCallback(path_manager::PathManagerConfig& config,
                           uint32_t level);
};

// ── 初始化 ─────────────────────────────────────────────────
bool PathManager::init() {
  // 参数
  pnh_.param<std::string>("yaml_file_path", yaml_file_path_,
                          ros::package::getPath("robot_bringup") + "/config/paths.yaml");
  pnh_.param<double>("publish_rate_hz", publish_rate_hz_, 10.0);

  // 发布者
  path_pub_ = nh_.advertise<path_manager::PathPoint>("/path_points", 10);

  // 服务
  load_srv_   = nh_.advertiseService("/load_path",   &PathManager::handleLoadPath,   this);
  select_srv_ = nh_.advertiseService("/select_path", &PathManager::handleSelectPath, this);
  next_srv_   = nh_.advertiseService("/next_point",  &PathManager::handleNextPoint,  this);

  // 动态参数
  cfg_cb_ = boost::bind(&PathManager::reconfigureCallback, this, _1, _2);
  cfg_server_.setCallback(cfg_cb_);

  // 加载路径
  if (!loadPathsFromFile(yaml_file_path_)) {
    ROS_ERROR("[path_manager] 初始加载 paths.yaml 失败");
    return false;
  }

  // 定时发布
  pub_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_hz_),
                               &PathManager::publishTimerCallback, this);

  ROS_INFO("[path_manager] 初始化完成，已加载 %zu 条路径", paths_.size());
  return true;
}

// ── 从参数服务器读取路径 ──────────────────────────────────
bool PathManager::loadPathsFromParam() {
  paths_.clear();
  active_path_idx_  = -1;
  active_point_idx_ = 0;

  XmlRpc::XmlRpcValue xml_paths;
  if (!nh_.getParam("/path_manager/paths", xml_paths)) {
    ROS_ERROR("[path_manager] 参数 /path_manager/paths 不存在");
    return false;
  }

  if (xml_paths.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("[path_manager] /path_manager/paths 不是数组类型");
    return false;
  }

  for (int i = 0; i < xml_paths.size(); ++i) {
    const XmlRpc::XmlRpcValue& entry = xml_paths[i];
    if (!entry.hasMember("name") || !entry.hasMember("points")) {
      ROS_WARN("[path_manager] 跳过格式不正确的路径条目 %d", i);
      continue;
    }

    PathDef pd;
    pd.name = static_cast<std::string>(entry["name"]);

    const XmlRpc::XmlRpcValue& pts = entry["points"];
    if (pts.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("[path_manager] 路径 '%s' 的 points 不是数组", pd.name.c_str());
      continue;
    }

    for (int j = 0; j < pts.size(); ++j) {
      const XmlRpc::XmlRpcValue& p = pts[j];
      PointDef pt;
      pt.x         = static_cast<double>(p["x"]);
      pt.y         = static_cast<double>(p["y"]);
      pt.tolerance = p.hasMember("tolerance")
                       ? static_cast<double>(p["tolerance"]) : 0.05;
      pd.points.push_back(pt);
    }

    if (pd.points.empty()) {
      ROS_WARN("[path_manager] 路径 '%s' 没有路径点，跳过", pd.name.c_str());
      continue;
    }

    paths_.push_back(pd);
  }

  if (paths_.empty()) {
    ROS_ERROR("[path_manager] 未加载到任何有效路径");
    return false;
  }

  ROS_INFO("[path_manager] 从参数服务器加载了 %zu 条路径", paths_.size());
  for (const auto& p : paths_) {
    ROS_INFO("  - %s (%zu 个点)", p.name.c_str(), p.points.size());
  }
  return true;
}

// ── 从文件加载 ─────────────────────────────────────────────
bool PathManager::loadPathsFromFile(const std::string& file_path) {
  // 使用 rosparam load 将 YAML 加载到 /path_manager 命名空间
  std::ostringstream cmd;
  cmd << "rosparam load " << file_path << " /path_manager";
  int ret = std::system(cmd.str().c_str());
  if (ret != 0) {
    ROS_ERROR("[path_manager] rosparam load 失败: %s", file_path.c_str());
    return false;
  }

  // 短暂等待参数生效
  ros::Duration(0.1).sleep();

  return loadPathsFromParam();
}

// ── 定时发布当前路径点 ─────────────────────────────────────
void PathManager::publishTimerCallback(const ros::TimerEvent& /*event*/) {
  if (active_path_idx_ < 0 ||
      active_path_idx_ >= static_cast<int>(paths_.size())) {
    return;  // 没有选中路径
  }

  const PathDef& path = paths_[active_path_idx_];
  if (active_point_idx_ < 0 ||
      active_point_idx_ >= static_cast<int>(path.points.size())) {
    return;  // 路径已完成
  }

  path_manager::PathPoint msg;

  if (override_enabled_) {
    msg.x         = override_x_;
    msg.y         = override_y_;
    msg.tolerance = override_tolerance_;
  } else {
    const PointDef& pt = path.points[active_point_idx_];
    msg.x         = pt.x;
    msg.y         = pt.y;
    msg.tolerance = pt.tolerance;
  }

  path_pub_.publish(msg);
}

// ── /load_path 服务 ────────────────────────────────────────
bool PathManager::handleLoadPath(
    path_manager::LoadPath::Request&  req,
    path_manager::LoadPath::Response& res) {
  std::string file = req.file_path.empty() ? yaml_file_path_ : req.file_path;

  if (loadPathsFromFile(file)) {
    yaml_file_path_ = file;
    res.success = true;
    res.message = "成功加载: " + file;
    ROS_INFO("[path_manager] %s", res.message.c_str());
  } else {
    res.success = false;
    res.message = "加载失败: " + file;
    ROS_ERROR("[path_manager] %s", res.message.c_str());
  }
  return true;
}

// ── /select_path 服务 ──────────────────────────────────────
bool PathManager::handleSelectPath(
    path_manager::SelectPath::Request&  req,
    path_manager::SelectPath::Response& res) {
  for (size_t i = 0; i < paths_.size(); ++i) {
    if (paths_[i].name == req.path_name) {
      active_path_idx_  = static_cast<int>(i);
      active_point_idx_ = 0;
      override_enabled_ = false;  // 切换路径时清除覆盖

      res.success = true;
      res.message = "已选择路径: " + req.path_name
                  + " (" + std::to_string(paths_[i].points.size()) + " 个点)";
      ROS_INFO("[path_manager] %s", res.message.c_str());
      return true;
    }
  }

  res.success = false;
  res.message = "未找到路径: " + req.path_name;
  ROS_WARN("[path_manager] %s", res.message.c_str());
  return true;
}

// ── /next_point 服务 ───────────────────────────────────────
bool PathManager::handleNextPoint(
    std_srvs::Trigger::Request&  /*req*/,
    std_srvs::Trigger::Response& res) {
  if (active_path_idx_ < 0 ||
      active_path_idx_ >= static_cast<int>(paths_.size())) {
    res.success = false;
    res.message = "没有选中路径";
    return true;
  }

  const PathDef& path = paths_[active_path_idx_];
  if (active_point_idx_ + 1 >= static_cast<int>(path.points.size())) {
    res.success = true;
    res.message = "已到达路径终点 (点 " +
                  std::to_string(active_point_idx_ + 1) + "/" +
                  std::to_string(path.points.size()) + ")";
    ROS_INFO("[path_manager] %s", res.message.c_str());
    return true;  // 不自动重置，允许调用方决定是否循环
  }

  active_point_idx_++;
  override_enabled_ = false;  // 切换到新点后清除覆盖

  res.success = true;
  res.message = "前进到点 " + std::to_string(active_point_idx_ + 1) +
                "/" + std::to_string(path.points.size()) +
                " (x=" + std::to_string(path.points[active_point_idx_].x) +
                ", y=" + std::to_string(path.points[active_point_idx_].y) + ")";
  ROS_INFO("[path_manager] %s", res.message.c_str());
  return true;
}

// ── dynamic_reconfigure 回调 ───────────────────────────────
void PathManager::reconfigureCallback(
    path_manager::PathManagerConfig& config,
    uint32_t /*level*/) {
  override_enabled_    = config.override_enabled;
  override_x_          = config.target_x;
  override_y_          = config.target_y;
  override_tolerance_  = config.target_tolerance;

  double old_rate = publish_rate_hz_;
  publish_rate_hz_ = config.publish_rate_hz;

  if (std::abs(publish_rate_hz_ - old_rate) > 0.01) {
    pub_timer_.stop();
    pub_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_hz_),
                                 &PathManager::publishTimerCallback, this);
  }

  if (override_enabled_) {
    ROS_INFO_THROTTLE(2.0, "[path_manager] 动态覆盖: (%.3f, %.3f) tol=%.3f",
                      override_x_, override_y_, override_tolerance_);
  }
}

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "path_manager_node");

  PathManager pm;
  if (!pm.init()) {
    ROS_FATAL("[path_manager] 初始化失败，退出");
    return 1;
  }

  ros::spin();
  return 0;
}
