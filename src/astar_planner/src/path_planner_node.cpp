#include <memory>
#include <vector>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "astar_planner/astar.hpp"

using namespace std::chrono_literals;

class PathPlannerNode : public rclcpp::Node
{
public:
  PathPlannerNode()
  : Node("path_planner_node")
  {
    // Declare parameters
    this->declare_parameter<double>("resolution", 1.0);
    
    resolution_ = this->get_parameter("resolution").as_double();
    
    // Initialize
    has_map_ = false;
    has_goal_ = false;
    has_current_pose_ = false;
    goal_reached_ = false;
    
    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&PathPlannerNode::mapCallback, this, std::placeholders::_1));
    
    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/go1_pose", 10,
      std::bind(&PathPlannerNode::currentPoseCallback, this, std::placeholders::_1));
    
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));
    
    // Publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_markers", 10);
    goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);
    
    RCLCPP_INFO(this->get_logger(), "Path Planner Node initialized");
    RCLCPP_INFO(this->get_logger(), "Use RViz2 '2D Goal Pose' tool to set a goal");
  }

private:
  // [추가] 쿼터니언을 Yaw(라디안) 각도로 변환하는 헬퍼 함수
  double getYaw(const geometry_msgs::msg::Quaternion & q)
  {
    // Roll-Pitch-Yaw 중 Yaw(z축 회전)만 계산
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_msg_ = msg;
    
    // Convert OccupancyGrid to 2D vector
    int width = msg->info.width;
    int height = msg->info.height;
    
    map_grid_.clear();
    map_grid_.resize(height, std::vector<int>(width));
    
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int index = y * width + x;
        // OccupancyGrid: -1 (unknown), 0 (free), 100 (occupied)
        // Convert to binary: 0 (free), 1 (occupied)
        if (msg->data[index] > 50 || msg->data[index] < 0) {
          map_grid_[y][x] = 1;  // obstacle
        } else {
          map_grid_[y][x] = 0;  // free
        }
      }
    }
    
    astar_.setMap(map_grid_);
    
    if (!has_map_) {
      has_map_ = true;
      RCLCPP_INFO(this->get_logger(), "Map received: %dx%d", width, height);
    }
  }
  
  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!has_current_pose_) {
      has_current_pose_ = true;
      current_pose_ = *msg;
      previous_pose_ = *msg;
      RCLCPP_INFO(this->get_logger(), "Robot position initialized at (%.2f, %.2f)",
        current_pose_.pose.position.x, current_pose_.pose.position.y);
      return;
    }
    
    // Check if robot position actually changed
    double dx = msg->pose.position.x - previous_pose_.pose.position.x;
    double dy = msg->pose.position.y - previous_pose_.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // Only replan if position changed significantly (moved to new grid cell)
    if (distance < 0.5) {  // Threshold for considering position unchanged
      return;
    }
    
    current_pose_ = *msg;
    
    // Check if goal is reached
    if (has_goal_) {
      double goal_dx = current_pose_.pose.position.x - goal_pose_.pose.position.x;
      double goal_dy = current_pose_.pose.position.y - goal_pose_.pose.position.y;
      double goal_distance = std::sqrt(goal_dx * goal_dx + goal_dy * goal_dy);
      
      if (goal_distance < 0.3) {  // Goal reached threshold
        if (!goal_reached_) { // [수정] 목표 도달 시 상세 정보 출력
          // 각도(Yaw) 계산
          double target_yaw = getYaw(goal_pose_.pose.orientation);
          double current_yaw = getYaw(current_pose_.pose.orientation);
          
          RCLCPP_INFO(this->get_logger(), "✓ Goal Reached!");
          
          // 요청하신 비교 로그 출력 (보기 좋게 정렬)
          RCLCPP_INFO(this->get_logger(), 
            "  [Target] x: %.3f, y: %.3f, yaw: %.3f (rad)", 
            goal_pose_.pose.position.x, goal_pose_.pose.position.y, target_yaw);
            
          RCLCPP_INFO(this->get_logger(), 
            "  [Actual] x: %.3f, y: %.3f, yaw: %.3f (rad)", 
            current_pose_.pose.position.x, current_pose_.pose.position.y, current_yaw);
            
          RCLCPP_INFO(this->get_logger(), 
            "  [Error ] dx: %.3f, dy: %.3f, dyaw: %.3f", 
            goal_dx, goal_dy, target_yaw - current_yaw);
          goal_reached_ = true;
        }
        return;  // Don't replan if goal is reached
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Robot moved to (%.2f, %.2f)", 
      current_pose_.pose.position.x, current_pose_.pose.position.y);
    
    // Store current position as previous for next comparison
    previous_pose_ = current_pose_;
    
    // Replan path whenever robot position changes (and we have a goal)
    if (has_map_ && has_goal_ && !goal_reached_) {
      replanPath();
    }
  }
  
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_ = *msg;
    has_goal_ = true;
    goal_reached_ = false;  // Reset goal reached flag for new goal
    
    RCLCPP_INFO(this->get_logger(), 
      "New goal received: (%.2f, %.2f)", 
      goal_pose_.pose.position.x, 
      goal_pose_.pose.position.y);
    
    // Publish goal marker for visualization
    publishGoalMarker();
    
    // Plan path immediately when goal is set
    if (has_map_ && has_current_pose_) {
      replanPath();
    }
  }
  
  void replanPath()
  {
    if (!has_map_ || !has_current_pose_ || !has_goal_) {
      return;
    }
    
    // Convert world coordinates to grid coordinates
    astar_planner::GridCell start = worldToGrid(
      current_pose_.pose.position.x,
      current_pose_.pose.position.y);
    
    astar_planner::GridCell goal = worldToGrid(
      goal_pose_.pose.position.x,
      goal_pose_.pose.position.y);
    
    // Find path using A*
    auto path_cells = astar_.findPath(start, goal);
    
    if (path_cells.empty()) {
      RCLCPP_WARN(this->get_logger(), "No path found!");
      return;
    }
    
    // Convert grid path to ROS Path message
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";
    
    // First waypoint is always current pose
    geometry_msgs::msg::PoseStamped first_pose;
    first_pose.header.stamp = this->now();
    first_pose.header.frame_id = "map";
    first_pose.pose = current_pose_.pose;
    path_msg.poses.push_back(first_pose);
    
    // Add rest of the path (skip first cell if it's same as current position)
    for (size_t i = 0; i < path_cells.size(); ++i) {
      const auto& cell = path_cells[i];
      
      auto world_pos = gridToWorld(cell.x, cell.y);
      
      // Skip if this waypoint is too close to current position
      double dx = world_pos.first - current_pose_.pose.position.x;
      double dy = world_pos.second - current_pose_.pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      
      if (i == 0 && dist < 0.3) {
        continue;  // Skip first cell if robot is already there
      }
      
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "map";
      pose.pose.position.x = world_pos.first;
      pose.pose.position.y = world_pos.second;
      pose.pose.position.z = 0.0;
      // [수정 전] 무조건 0도(w=1.0)를 보라고 되어 있었음 -> MPPI가 이것만 추종함
      // pose.pose.orientation.w = 1.0; 

      // [수정 후] 마지막 점이면 '목표 각도'를 넣고, 아니면 진행 방향을 넣거나 0도로 둠
      if (i == path_cells.size() - 1) {
          // 마지막 점: 사용자가 Rviz에서 지정한 Goal Orientation 적용!
          pose.pose.orientation = goal_pose_.pose.orientation;
      } else {
          // 중간 점: 그냥 0도로 두거나, 다음 점을 바라보게 계산 가능 (여기선 0도로 유지해도 무방)
          pose.pose.orientation.w = 1.0; 
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
      }
      
      path_msg.poses.push_back(pose);
    }
    
    path_pub_->publish(path_msg);
    
    // Publish visualization markers
    publishPathMarkers(path_cells);
    
    // Only log if path length changed significantly or first time
    static size_t last_path_size = 0;
    if (last_path_size == 0 || std::abs((int)path_cells.size() - (int)last_path_size) > 3) {
      RCLCPP_INFO(this->get_logger(), "Path updated: %zu waypoints", path_cells.size());
      last_path_size = path_cells.size();
    }
  }
  
  astar_planner::GridCell worldToGrid(double x, double y)
  {
    astar_planner::GridCell cell;
    
    // Apply origin offset from map
    double origin_x = map_msg_->info.origin.position.x;
    double origin_y = map_msg_->info.origin.position.y;
    double resolution = map_msg_->info.resolution;
    
    cell.x = static_cast<int>((x - origin_x) / resolution);
    cell.y = static_cast<int>((y - origin_y) / resolution);
    
    return cell;
  }
  
  std::pair<double, double> gridToWorld(int x, int y)
  {
    double origin_x = map_msg_->info.origin.position.x;
    double origin_y = map_msg_->info.origin.position.y;
    double resolution = map_msg_->info.resolution;
    
    double world_x = origin_x + (x + 0.5) * resolution;
    double world_y = origin_y + (y + 0.5) * resolution;
    
    return {world_x, world_y};
  }
  
  void publishPathMarkers(const std::vector<astar_planner::GridCell>& path)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Create line strip for path
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->now();
    line_marker.ns = "path";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.1;  // Line width
    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;
    
    for (const auto& cell : path) {
      geometry_msgs::msg::Point p;
      auto world_pos = gridToWorld(cell.x, cell.y);
      p.x = world_pos.first;
      p.y = world_pos.second;
      p.z = 0.1;
      line_marker.points.push_back(p);
    }
    
    marker_array.markers.push_back(line_marker);
    viz_pub_->publish(marker_array);
  }
  
  void publishGoalMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = goal_pose_.pose.position.x;
    marker.pose.position.y = goal_pose_.pose.position.y;
    marker.pose.position.z = 0.5;
    marker.pose.orientation = goal_pose_.pose.orientation;
    
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;
    
    goal_marker_pub_->publish(marker);
  }
  
  // ROS objects
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  
  // State variables
  bool has_map_;
  bool has_goal_;
  bool has_current_pose_;
  bool goal_reached_;
  
  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped previous_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  
  std::vector<std::vector<int>> map_grid_;
  astar_planner::AStar astar_;
  
  // Parameters
  double resolution_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
