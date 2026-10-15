// map_saver.cpp — saves the live SLAM map in 3 formats with retry & file
// verification.  C++ port of rover_bringup/map_saver.py.
//
// Trigger:
//   service /save_all_maps  (std_srvs/srv/Trigger)  — manual
//
//   The auto-save-on-/init_complete trigger was removed in the 2026-08-15
//   cleanup: it depended on boundary_detector (deleted for CPU reasons) and
//   was no longer used.
//
// Output (DIR=map_dir, NAME=map_basename):
//   NAME.posegraph + NAME.data         (slam_toolbox SerializePoseGraph)
//   NAME.yaml      + NAME.pgm          (slam_toolbox SaveMap)
//   NAME_nav2.yaml + NAME_nav2.pgm     (nav2_map_server map_saver_cli)
//
// THREADING: save_all() blocks (it waits for services and polls the filesystem)
// and is called from a service callback, so the node runs on a
// MultiThreadedExecutor with a reentrant callback group.  On a single-threaded
// executor the blocking wait would deadlock against the very service responses
// it is waiting for.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "slam_toolbox/srv/save_map.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"

using namespace std::chrono_literals;

namespace
{
bool is_file(const std::string & path)
{
  struct stat st;
  return stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

double now_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class MapSaver : public rclcpp::Node
{
public:
  MapSaver()
  : Node("map_saver")
  {
    map_dir_ = this->declare_parameter<std::string>(
      "map_dir", "/workspaces/isaac_ros-dev/slm_maps");
    this->declare_parameter<std::string>("map_basename", "live_map");
    max_retries_      = this->declare_parameter<int>("max_retries", 3);
    verify_timeout_s_ = this->declare_parameter<double>("verify_timeout_s", 4.0);
    retry_backoff_s_  = this->declare_parameter<double>("retry_backoff_s", 0.5);

    // mkdir -p on map_dir
    if (std::system(("mkdir -p '" + map_dir_ + "'").c_str()) != 0) {
      RCLCPP_ERROR(this->get_logger(), "could not create %s", map_dir_.c_str());
    }

    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    srv_ = this->create_service<std_srvs::srv::Trigger>(
      "save_all_maps",
      std::bind(
        &MapSaver::on_save_request, this,
        std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), cb_group_);

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/map_saver/last_save_status", 10);

    save_map_cli_ = this->create_client<slam_toolbox::srv::SaveMap>(
      "/slam_toolbox/save_map", rclcpp::ServicesQoS().get_rmw_qos_profile(), cb_group_);
    serialize_cli_ = this->create_client<slam_toolbox::srv::SerializePoseGraph>(
      "/slam_toolbox/serialize_map", rclcpp::ServicesQoS().get_rmw_qos_profile(), cb_group_);

    RCLCPP_INFO(
      this->get_logger(),
      "map_saver ready  dir=%s  basename=%s  "
      "(manual save only: ros2 service call /save_all_maps std_srvs/srv/Trigger)",
      map_dir_.c_str(),
      this->get_parameter("map_basename").as_string().c_str());
  }

private:
  void on_save_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    const auto result = save_all();
    resp->success = result.first;
    resp->message = result.second;
  }

  std::string full_path()
  {
    return map_dir_ + "/" + this->get_parameter("map_basename").as_string();
  }

  void publish_status(bool ok, const std::string & msg)
  {
    std_msgs::msg::String s;
    s.data = (ok ? "OK: " : "FAIL: ") + msg;
    status_pub_->publish(s);
    if (ok) {
      RCLCPP_INFO(this->get_logger(), "%s", s.data.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "%s", s.data.c_str());
    }
  }

  // Run `action`, then poll `verify` for up to verify_timeout_s.  Repeat up to
  // max_retries times.  Identical control flow to the Python version.
  bool retry(
    const std::function<void()> & action,
    const std::function<bool()> & verify,
    const std::string & label)
  {
    for (int attempt = 1; attempt <= max_retries_; ++attempt) {
      try {
        action();
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          this->get_logger(), "%s attempt %d threw: %s",
          label.c_str(), attempt, e.what());
      }
      const double t0 = now_seconds();
      while (now_seconds() - t0 < verify_timeout_s_) {
        if (verify()) {
          return true;
        }
        std::this_thread::sleep_for(100ms);
      }
      RCLCPP_WARN(
        this->get_logger(), "%s attempt %d/%d: files missing, retrying",
        label.c_str(), attempt, max_retries_);
      std::this_thread::sleep_for(std::chrono::duration<double>(retry_backoff_s_));
    }
    return false;
  }

  void do_serialize()
  {
    if (!serialize_cli_->wait_for_service(2s)) {
      throw std::runtime_error("/slam_toolbox/serialize_map unavailable");
    }
    auto req = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();
    req->filename = full_path();
    serialize_cli_->async_send_request(req);
  }

  void do_savemap()
  {
    if (!save_map_cli_->wait_for_service(2s)) {
      throw std::runtime_error("/slam_toolbox/save_map unavailable");
    }
    auto req = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
    req->name.data = full_path();
    save_map_cli_->async_send_request(req);
  }

  void do_nav2_saver(const std::string & basepath)
  {
    const std::string cmd =
      "ros2 run nav2_map_server map_saver_cli -f '" + basepath + "' "
      ">/dev/null 2>&1";
    // The exit code is deliberately advisory — success is decided by the
    // file-existence check in retry(), exactly as the Python version did.
    // It still has to be consumed: GCC's warn_unused_result ignores (void).
    const int rc = std::system(cmd.c_str());
    if (rc != 0) {
      RCLCPP_DEBUG(this->get_logger(), "map_saver_cli exited %d", rc);
    }
  }

  std::pair<bool, std::string> save_all()
  {
    const std::string path = full_path();
    const std::string nav2_path = path + "_nav2";

    for (const char * ext : {".yaml", ".pgm", ".posegraph", ".data"}) {
      std::remove((path + ext).c_str());
      std::remove((nav2_path + ext).c_str());
    }

    std::vector<std::pair<std::string, bool>> results;

    const bool ok1 = retry(
      [this]() {do_serialize();},
      [path]() {return is_file(path + ".posegraph") && is_file(path + ".data");},
      "serialize_pose_graph");
    results.emplace_back("posegraph", ok1);

    const bool ok2 = retry(
      [this]() {do_savemap();},
      [path]() {return is_file(path + ".yaml") && is_file(path + ".pgm");},
      "slam_toolbox_save_map");
    results.emplace_back("slam_toolbox_yaml", ok2);

    const bool ok3 = retry(
      [this, nav2_path]() {do_nav2_saver(nav2_path);},
      [nav2_path]() {return is_file(nav2_path + ".yaml") && is_file(nav2_path + ".pgm");},
      "nav2_map_saver");
    results.emplace_back("nav2_yaml", ok3);

    bool all_ok = true;
    std::string msg;
    for (size_t i = 0; i < results.size(); ++i) {
      all_ok = all_ok && results[i].second;
      if (i) {
        msg += "; ";
      }
      msg += results[i].first + "=" + (results[i].second ? "OK" : "FAIL");
    }
    publish_status(all_ok, msg);
    return {all_ok, msg};
  }

  std::string map_dir_;
  int    max_retries_{3};
  double verify_timeout_s_{4.0};
  double retry_backoff_s_{0.5};

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr save_map_cli_;
  rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedPtr serialize_cli_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapSaver>();
  // Reentrant group + multiple threads: save_all() blocks while waiting on the
  // slam_toolbox services, and must not starve the executor that delivers them.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
