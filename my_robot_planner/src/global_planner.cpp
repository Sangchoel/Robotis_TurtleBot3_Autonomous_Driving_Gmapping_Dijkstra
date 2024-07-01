#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <queue>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <iostream>
#include <sys/stat.h>

class MyGlobalPlanner : public nav2_core::GlobalPlanner, public rclcpp::Node
{
public:
  MyGlobalPlanner()
  : rclcpp::Node("my_global_planner"), costmap_ros_(nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "MyGlobalPlanner constructor called");
    plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
      "global_planner/create_plan", std::bind(&MyGlobalPlanner::handlePlanRequest, this,
                                              std::placeholders::_1, std::placeholders::_2));
    
    // Path publisher
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("global_planner/path", 10);

    // Load the map
    loadMap("/home/osc/dev_ws/src/yaml/my_map.yaml");
  }

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    RCLCPP_INFO(this->get_logger(), "Configuring MyGlobalPlanner");
    costmap_ros_ = costmap_ros;
  }

  void cleanup() override
  {
    RCLCPP_INFO(this->get_logger(), "Cleaning up MyGlobalPlanner");
  }

  void activate() override
  {
    RCLCPP_INFO(this->get_logger(), "Activating MyGlobalPlanner");
  }

  void deactivate() override
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating MyGlobalPlanner");
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  cv::Mat map_image_;
  double origin_x_, origin_y_, resolution_;
  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr plan_service_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  struct Node {
    int x, y;
    double cost;

    Node(int x, int y, double cost) : x(x), y(y), cost(cost) {}
    Node() : x(0), y(0), cost(0.0) {}

    bool operator>(const Node& other) const {
      return cost > other.cost;
    }
  };

  void loadMap(const std::string& yaml_file)
  {
    RCLCPP_INFO(this->get_logger(), "Loading map from: %s", yaml_file.c_str());

    YAML::Node map_yaml;
    try {
      map_yaml = YAML::LoadFile(yaml_file);
    } catch (const YAML::BadFile& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", yaml_file.c_str());
      return;
    }

    std::string image_file = map_yaml["image"].as<std::string>();

    // 절대 경로로 변경
    std::string image_path = "/home/osc/dev_ws/src/yaml/" + image_file;

    RCLCPP_INFO(this->get_logger(), "Map image path: %s", image_path.c_str());

    // Check if the image file exists
    struct stat buffer;
    if (stat(image_path.c_str(), &buffer) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Map image file does not exist: %s", image_path.c_str());
      return;
    }

    resolution_ = map_yaml["resolution"].as<double>();
    origin_x_ = map_yaml["origin"][0].as<double>();
    origin_y_ = map_yaml["origin"][1].as<double>();

    map_image_ = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (map_image_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map image: %s", image_path.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Map loaded successfully: %s", image_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Map size: (%d, %d)", map_image_.cols, map_image_.rows);
    }
  }

  void handlePlanRequest(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Handling plan request from (%.2f, %.2f) to (%.2f, %.2f)",
                request->start.pose.position.x, request->start.pose.position.y,
                request->goal.pose.position.x, request->goal.pose.position.y);
    response->plan = createPlan(request->start, request->goal);
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    RCLCPP_INFO(this->get_logger(), "Creating plan");
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "map";

    if (map_image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Map image is empty. Cannot create plan.");
      return path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> poses = dijkstra(start, goal);
    path.poses = poses;
    RCLCPP_INFO(this->get_logger(), "Plan created with %zu poses", poses.size());

    // Publish the path
    path_publisher_->publish(path);

    return path;
  }

std::vector<geometry_msgs::msg::PoseStamped> dijkstra(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_INFO(this->get_logger(), "Starting Dijkstra algorithm");
  std::vector<geometry_msgs::msg::PoseStamped> path;
  int width = map_image_.cols;
  int height = map_image_.rows;

  RCLCPP_INFO(this->get_logger(), "Map size: width=%d, height=%d", width, height);

  if (width == 0 || height == 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid map size. Cannot run Dijkstra algorithm.");
    return path;
  }

  unsigned int start_x = (start.pose.position.x - origin_x_) / resolution_;
  unsigned int start_y = (start.pose.position.y - origin_y_) / resolution_;
  unsigned int goal_x = (goal.pose.position.x - origin_x_) / resolution_;
  unsigned int goal_y = (goal.pose.position.y - origin_y_) / resolution_;

  RCLCPP_INFO(this->get_logger(), "Start world coordinates: x=%.2f, y=%.2f", start.pose.position.x, start.pose.position.y);
  RCLCPP_INFO(this->get_logger(), "Goal world coordinates: x=%.2f, y=%.2f", goal.pose.position.x, goal.pose.position.y);
  RCLCPP_INFO(this->get_logger(), "Start map coordinates: x=%u, y=%u", start_x, start_y);
  RCLCPP_INFO(this->get_logger(), "Goal map coordinates: x=%u, y=%u", goal_x, goal_y);

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
  std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));
  std::unordered_map<int, Node> all_nodes;
  std::unordered_map<int, int> came_from;
  
  open_list.emplace(start_x, start_y, 0.0);
  came_from[start_x + start_y * width] = start_x + start_y * width;

  int step_counter = 0;
  const int visualization_step = 1000;

  while (!open_list.empty()) {
    Node current = open_list.top();
    open_list.pop();

    if (visited[current.x][current.y]) continue;
    visited[current.x][current.y] = true;

    if (current.x == goal_x && current.y == goal_y) {
      break;
    }

    std::vector<std::pair<int, int>> neighbors = {
      {0, 1}, {1, 0}, {0, -1}, {-1, 0}
    };

    for (auto& offset : neighbors) {
      int new_x = current.x + offset.first;
      int new_y = current.y + offset.second;

      if (new_x < 0 || new_y < 0 || new_x >= width || new_y >= height) continue;

      unsigned char cost = map_image_.at<unsigned char>(new_y, new_x);
      if (cost == 0) {
        continue;
      }

      double new_cost = current.cost + 1.0;

      if (all_nodes.find(new_x + new_y * width) == all_nodes.end() || new_cost < all_nodes[new_x + new_y * width].cost) {
        all_nodes[new_x + new_y * width] = Node(new_x, new_y, new_cost);
        open_list.emplace(new_x, new_y, new_cost);
        came_from[new_x + new_y * width] = current.x + current.y * width;
      }
    }

    if (step_counter % visualization_step == 0) {
      RCLCPP_INFO(this->get_logger(), "Visited nodes: %d, Open list size: %zu", step_counter, open_list.size());
    }
    step_counter++;
  }

  if (came_from.find(goal_x + goal_y * width) == came_from.end()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to find a path to the goal");
    return path;
  }

  int current = goal_x + goal_y * width;
  while (current != start_x + start_y * width) {
    int x = current % width;
    int y = current / width;
    geometry_msgs::msg::PoseStamped pose;
    double wx = x * resolution_ + origin_x_;
    double wy = y * resolution_ + origin_y_;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;

    // Calculate orientation
    if (!path.empty()) {
      double dx = pose.pose.position.x - path.back().pose.position.x;
      double dy = pose.pose.position.y - path.back().pose.position.y;
      pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), std::atan2(dy, dx)));
    }

    path.push_back(pose);
    current = came_from[current];
  }

  std::reverse(path.begin(), path.end());
  RCLCPP_INFO(this->get_logger(), "Dijkstra algorithm finished");

  return path;
}

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("global_planner"), "Starting MyGlobalPlanner node");
  auto node = std::make_shared<MyGlobalPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
