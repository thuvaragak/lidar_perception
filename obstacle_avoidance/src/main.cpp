#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstddef>
#include <chrono>

#include "sensors/lidar.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include "obstacle_avoidance/msg/control.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <tf2/LinearMath/Quaternion.h>
// #include "lidar_obstacle_detector.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h> 
#include <pcl/common/centroid.h>

#define M_PI 3.14159265358979323846
using std::placeholders::_1;

struct Waypoint2D
{
  double x;
  double y;
};

struct NearestPointResult
{
  int    index;      // index of segment start waypoint (like best_idx)
  double proj_x;     // projected point x on segment
  double proj_y;     // projected point y on segment
  double distance;   // distance from (px, py) to projected point
  double param;      // segment parameter in [0, 1]
};

struct ObstacleInfo {
  float x;
  float y;
  float dist;
};


// ===================== Main node ===========================

class LaserSensor : public rclcpp::Node
{
public:
  LaserSensor()
  : Node("laser_sensor")
  {
    // ---- TF buffer + listener (only for LiDAR) -----------------
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- Subscriptions ------------------------------------------
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/data", 10, std::bind(&LaserSensor::pointcloud_callback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, std::bind(&LaserSensor::imu_callback, this, _1));
    
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/vel/data", 10, std::bind(&LaserSensor::vel_callback, this, _1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/global_plan", 10, std::bind(&LaserSensor::path_callback, this, _1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/vehicle/pose", 10, std::bind(&LaserSensor::pose_callback, this, _1));

    // ---- Publisher ----------------------------------------------
    cmd_pub_ = this->create_publisher<obstacle_avoidance::msg::Control>("/control", 10);
    bbox_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_bboxes", 10);

    // ---- Control loop timer (DWA) -------------------------------
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),   // 20 Hz
      std::bind(&LaserSensor::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "LaserSensor (LiDAR + DWA v,w) node started");
  }

private:
  // =================== ROS callbacks =======================

  void pointcloud_callback( const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
  {
    
    try {
        if (!tf_buffer_) {
          RCLCPP_WARN(this->get_logger(), "TF buffer not initialized");
          return;
        }
        // Convert ROS2 message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::fromROSMsg(*msg, *cloud);

        try{
          geometry_msgs::msg::TransformStamped transform_stamped;
          transform_stamped = tf_buffer_->lookupTransform(
          "base_link",                // target frame
          msg->header.frame_id,       // source frame
          tf2::TimePointZero);        // latest available

          sensor_msgs::msg::PointCloud2 transformed_cloud;
          tf2::doTransform(*msg, transformed_cloud, transform_stamped);
          pcl::fromROSMsg(transformed_cloud, *cloud);
          
        }
        catch (tf2::TransformException & ex) {
          RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
          return;
        }
        
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Processing %ld points", cloud->size());

        float voxel_size = 0.2f;   // 20 cm voxel

        Eigen::Vector4f minPoint(-10.0, -6.0, -2.5, 1.0);
        Eigen::Vector4f maxPoint( 30.0,  6.0,  2.0, 1.0);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);

        filtered_cloud = point_processor.FilterCloud(
            cloud,
            voxel_size,
            minPoint,
            maxPoint
        );

        auto segmented = point_processor.SegmentPlane(filtered_cloud, 100, 0.2f);

        // segmented.first  = obstacles
        // segmented.second = plane (ground)
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cloud = segmented.first;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud    = segmented.second;

        cloudClusters = point_processor.Clustering(segmented.first, 0.5, 15, 400);

        visualization_msgs::msg::MarkerArray marker_array;
        auto stamp = rclcpp::Time(msg->header.stamp);

        int id = 0;
        for (const auto &cluster : cloudClusters)
        {
          if (cluster->empty()) continue;

          Eigen::Vector4f min_pt, max_pt;
          pcl::getMinMax3D(*cluster, min_pt, max_pt);

          marker_array.markers.push_back(
              makeBBoxMarker(min_pt, max_pt, "base_link", stamp, id++)
          );
        }
        bbox_pub_->publish(marker_array);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_imu_ = *msg;
    have_imu_ = true;
  }
  
  void vel_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    latest_vel_ = *msg;
    have_vel_ = true;

    // We don't strictly need current v,w here since we use a simple window.
    current_linear_vel_  = msg->x;
    current_angular_vel_ = msg->z;
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    global_path_ = *msg;
    have_path_ = !global_path_.poses.empty();
    // RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());
  }

  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    latest_pose_ = *msg;
    have_pose_ = true;
  }

  // =================== Obstacle Avoidance ===================

  static visualization_msgs::msg::Marker makeBBoxMarker(
    const Eigen::Vector4f &min_pt,
    const Eigen::Vector4f &max_pt,
    const std::string &frame_id,
    const rclcpp::Time &stamp,
    int id)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id;
      m.header.stamp = stamp;

      m.ns = "cluster_bboxes";
      m.id = id;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;

      // center
      m.pose.position.x = 0.5f * (min_pt.x() + max_pt.x());
      m.pose.position.y = 0.5f * (min_pt.y() + max_pt.y());
      m.pose.position.z = 0.5f * (min_pt.z() + max_pt.z());
      m.pose.orientation.w = 1.0;

      // size
      m.scale.x = std::max(0.01f, max_pt.x() - min_pt.x());
      m.scale.y = std::max(0.01f, max_pt.y() - min_pt.y());
      m.scale.z = std::max(0.01f, max_pt.z() - min_pt.z());

      // color: RED (obstacles)
      m.color.a = 0.35f;
      m.color.r = 1.0f;
      m.color.g = 0.0f;
      m.color.b = 0.0f;

      // prevent stale markers
      m.lifetime = rclcpp::Duration::from_seconds(0.1);

      return m;
    }

  static ObstacleInfo getClosestFrontObstacle(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters)
  {
    ObstacleInfo best{0,0, std::numeric_limits<float>::infinity()};

    for (const auto& c : clusters)
    {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*c, centroid);

      float x = centroid.x();
      float y = centroid.y();

      // keep only obstacles in front
      if (x < 0.5f) continue;

      float d = std::hypot(x, y);
      if (d < best.dist) {
        best = {x, y, d};
      }
    }
    return best;
  }
  
  void applyAvoidance(obstacle_avoidance::msg::Control& cmd, const ObstacleInfo& obs,float stop_dist = 6.0f,
                      float steer_gain = 0.8f, float corridor_half_width = 2.0f)
  {
    if (!std::isfinite(obs.dist)) return;           // no obstacle
    // if (obs.x < 0.5f) return;                       // ignore behind/too close to origin
    if (std::abs(obs.y) > corridor_half_width) {    // far to the side → ignore for now
      return;
    }

    // 0..1 (1 = far, 0 = very close)
    float scale = std::clamp(obs.dist / stop_dist, 0.0f, 1.0f);

    // -------- Speed / brake policy ----------
    if (obs.dist < stop_dist)
    {
      // slow down as it gets closer
      cmd.throttle = std::min(cmd.throttle, 0.5f * scale);

      if (obs.dist < 2.5f) {
        cmd.throttle = 0.0f;
        cmd.brake = std::max(cmd.brake, 0.9f);
      } else if (obs.dist < 4.0f) {
        cmd.throttle = std::min(cmd.throttle, 0.2f);
        cmd.brake = std::max(cmd.brake, 0.6f);
      } else {
        cmd.brake = std::max(cmd.brake, 0.2f);
      }
    }
    else if (obs.dist < 9.0f && cmd.throttle > 0.3f)
    {
      // mild caution zone
      cmd.brake = std::max(cmd.brake, 0.0f);
      cmd.throttle = std::min(cmd.throttle, 0.3f);
    }

    // -------- Steering away ----------
    // obstacle on left (y>0) -> steer right (negative)
    // obstacle on right (y<0) -> steer left (positive)
    float away = (obs.y > 0.0f) ? -1.0f : 1.0f;

    // Stronger steering when closer
    float steer_push = steer_gain * (1.0f - scale);

    // If obstacle nearly centered, don't over-steer; just slow down
    if (std::abs(obs.y) < 0.5f) {
      steer_push *= 0.3f;
    }

    cmd.steer = std::clamp(cmd.steer + away * steer_push, -1.0f, 1.0f);
  }
  // =================== Pure Pursuit ==================
  NearestPointResult findNearestPoint( double px, double py) {

    NearestPointResult result;
    result.index    = -1;
    result.proj_x   = 0.0;
    result.proj_y   = 0.0;
    result.distance = std::numeric_limits<double>::infinity();
    result.param    = 0.0;

    if (global_path_.poses.size() < 2) {
      return result;  // not enough points to form a segment
    }

    double min_dist_sq = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i + 1 < global_path_.poses.size(); ++i){

      const auto & wp1 = global_path_.poses[i].pose.position;
      const auto & wp2 = global_path_.poses[i + 1].pose.position; 
      double dx = wp2.x - wp1.x;
      double dy = wp2.y - wp1.y;

      double seg_len_sq = dx * dx + dy * dy;
      double ptx = px - wp1.x;
      double pty = py - wp1.y;

      double param, proj_x, proj_y;

      if (seg_len_sq < 1e-6) {
        // Degenerate segment, treat as a point
        param  = 0.0;
        proj_x = wp1.x;
        proj_y = wp1.y;
      } else {
        // Project point onto segment in [0, 1]
        double dot = ptx * dx + pty * dy;
        param = dot / seg_len_sq;
        if (param < 0.0) param = 0.0;
        if (param > 1.0) param = 1.0;
        
        proj_x = wp1.x + param * dx;
        proj_y = wp1.y + param * dy;
      }

      double dpx = px - proj_x;
      double dpy = py - proj_y;
      double dist_sq = dpx * dpx + dpy * dpy;

      if (dist_sq < min_dist_sq) {
        min_dist_sq      = dist_sq;
        result.index     = static_cast<int>(i);
        result.proj_x    = proj_x;
        result.proj_y    = proj_y;
        result.distance  = std::sqrt(dist_sq);
        result.param     = param;
      }
    }
    return result;

  }

  Waypoint2D getTargetPoint(double lookahead_dist, const NearestPointResult & np)
  {
    Waypoint2D target_wp;
    if(np.index == -1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "DWA (v,w): no nearest point found on path");
    }
    else{
      // Moving along path
      int cur_idx = np.index;
      double remain_dist = 10.0 + np.distance;

      while (cur_idx+1 < global_path_.poses.size() && remain_dist > 0.0) {
        const auto & wp1 = global_path_.poses[cur_idx].pose.position;
        const auto & wp2 = global_path_.poses[cur_idx + 1].pose.position; 
        double dx = wp2.x - wp1.x;
        double dy = wp2.y - wp1.y;
        double seg_len = std::hypot(dx, dy);

        if (remain_dist > seg_len) {
          remain_dist -= seg_len;
          cur_idx++;
          target_wp.x = global_path_.poses.back().pose.position.x;
          target_wp.y = global_path_.poses.back().pose.position.y;
        } else {
          double ratio = remain_dist / seg_len;
          double target_x = wp1.x + ratio * dx;
          double target_y = wp1.y + ratio * dy;
          target_wp.x = target_x;
          target_wp.y = target_y;
          return target_wp;
        }
      }
    }
    
    return target_wp;
  }

  double getDesiredDirection(double target_x, double target_y)
  {
    double yaw = latest_pose_.orientation.z * M_PI / 180.0;
    double dx = target_x - latest_pose_.position.x;
    double dy = target_y - latest_pose_.position.y;

    double x_rel = std::cos(yaw) * dx + std::sin(yaw) * dy;
    double y_rel = -std::sin(yaw) * dx + std::cos(yaw) * dy;

    double desired_heading = std::atan2(y_rel, x_rel);
    return desired_heading;
  }

  // =================== Control core ============================

  void controlLoop()
  {
    if (!have_path_) {
      return;
    }

    // 1) Implement Pure Pursuit
    NearestPointResult np = findNearestPoint(latest_pose_.position.x, latest_pose_.position.y);
    Waypoint2D target_wp = getTargetPoint(10.0, np);  // lookahead 10 m
    double desired_heading = getDesiredDirection(target_wp.x, target_wp.y);
    
    // 2) Lidar Obstacle Processing
    RCLCPP_INFO(this->get_logger(), "Obstacles detected: %zu ", cloudClusters.size());
    ObstacleInfo closest_obstacle = getClosestFrontObstacle(cloudClusters);
    RCLCPP_INFO(this->get_logger(), "Closest obstacle at (%.2f, %.2f), dist=%.2f m",
      closest_obstacle.x, closest_obstacle.y, closest_obstacle.dist);

    // 3) Control command calculation for steering and speed
    obstacle_avoidance::msg::Control cmd;
    cmd.throttle = 0.4f;  // default throttle
    cmd.steer    = static_cast<float>(std::clamp(desired_heading, -1.0, 1.0));
    cmd.brake    = 0.0;

    // 4) Obstacle avoidance adjustment
    applyAvoidance(cmd, closest_obstacle, 6.0f, 0.8f);
    RCLCPP_INFO(this->get_logger(), "Control command: throttle=%.2f, steer=%.2f, brake=%.2f",
      cmd.throttle, cmd.steer, cmd.brake);  
    cmd_pub_->publish(cmd);
  }

  // =================== Members ============================

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr   vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr           path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr      pose_sub_;

  rclcpp::Publisher<obstacle_avoidance::msg::Control>::SharedPtr        cmd_pub_;
  rclcpp::TimerBase::SharedPtr                                   control_timer_;

  sensor_msgs::msg::Imu          latest_imu_;
  geometry_msgs::msg::Vector3    latest_vel_;
  nav_msgs::msg::Path            global_path_;
  geometry_msgs::msg::Pose       latest_pose_;
  bool have_pose_      = false;

  bool have_imu_       = false;
  bool have_vel_       = false;
  bool have_path_      = false;
  bool have_obstacles_ = false;

  double current_linear_vel_  = 0.0;
  double current_angular_vel_ = 0.0;

  ProcessPointClouds<pcl::PointXYZ> point_processor;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters;

};

// ======================= main ==============================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserSensor>());
  rclcpp::shutdown();
  return 0;
}
