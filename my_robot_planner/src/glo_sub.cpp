#include "rclcpp/rclcpp.hpp"  // ROS 2 핵심 라이브러리
#include "nav2_core/global_planner.hpp"  // Nav2 글로벌 플래너 인터페이스
#include "nav2_costmap_2d/costmap_2d.hpp"  // 코스트맵 라이브러리
#include "nav2_costmap_2d/costmap_2d_ros.hpp"  // 코스트맵 ROS 래퍼
#include "nav_msgs/msg/path.hpp"  // 경로 메시지
#include "geometry_msgs/msg/pose_stamped.hpp"  // PoseStamped 메시지
#include "geometry_msgs/msg/point.hpp"  // Point 메시지
#include "nav_msgs/srv/get_plan.hpp"  // GetPlan 서비스
#include <opencv2/opencv.hpp>  // OpenCV 라이브러리
#include <yaml-cpp/yaml.h>  // YAML 라이브러리
#include <queue>  // 다익스트라 알고리즘을 위한 우선순위 큐
#include <unordered_map>  // 노드를 추적하기 위한 언오더드 맵
#include <vector>  // 노드를 저장하기 위한 벡터
#include <fstream>  // 맵을 로드하기 위한 파일 스트림
#include <iostream>  // 입출력 스트림
#include <sys/stat.h>  // 파일 존재 여부를 확인하기 위한 파일 상태

// nav2_core::GlobalPlanner와 rclcpp::Node를 상속받은 MyGlobalPlanner 클래스 정의
class MyGlobalPlanner : public nav2_core::GlobalPlanner, public rclcpp::Node
{
public:
  MyGlobalPlanner()
  : rclcpp::Node("my_global_planner"), costmap_ros_(nullptr)
  {
    // 생성자 로깅
    RCLCPP_INFO(this->get_logger(), "MyGlobalPlanner 생성자 호출됨");

    // 플랜 요청을 위한 서비스 생성
    plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
      "global_planner/create_plan", std::bind(&MyGlobalPlanner::handlePlanRequest, this,
                                              std::placeholders::_1, std::placeholders::_2));
    
    // 경로 퍼블리셔 생성
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("global_planner/path", 10);

    // 지정된 파일에서 맵 로드
    loadMap("/home/osc/dev_ws/src/yaml/my_map.yaml");
  }

  // 글로벌 플래너를 설정하는 메소드
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    RCLCPP_INFO(this->get_logger(), "MyGlobalPlanner 설정 중");
    costmap_ros_ = costmap_ros;
  }

  // 정리 메소드
  void cleanup() override
  {
    RCLCPP_INFO(this->get_logger(), "MyGlobalPlanner 정리 중");
  }

  // 활성화 메소드
  void activate() override
  {
    RCLCPP_INFO(this->get_logger(), "MyGlobalPlanner 활성화 중");
  }

  // 비활성화 메소드
  void deactivate() override
  {
    RCLCPP_INFO(this->get_logger(), "MyGlobalPlanner 비활성화 중");
  }

private:
  // 코스트맵 ROS 래퍼
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // 맵을 위한 OpenCV 이미지
  cv::Mat map_image_;
  double origin_x_, origin_y_, resolution_;

  // ROS 2 서비스 및 퍼블리셔
  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr plan_service_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  // 다익스트라 알고리즘을 위한 노드 구조체
  struct Node {
    int x, y;
    double cost;

    Node(int x, int y, double cost) : x(x), y(y), cost(cost) {}
    Node() : x(0), y(0), cost(0.0) {}

    // 우선순위 큐를 위한 비교 연산자
    bool operator>(const Node& other) const {
      return cost > other.cost;
    }
  };

  // YAML 파일에서 맵을 로드하는 메소드
  void loadMap(const std::string& yaml_file)
  {
    RCLCPP_INFO(this->get_logger(), "맵을 로드 중: %s", yaml_file.c_str());

    YAML::Node map_yaml;
    try {
      map_yaml = YAML::LoadFile(yaml_file);
    } catch (const YAML::BadFile& e) {
      RCLCPP_ERROR(this->get_logger(), "YAML 파일 로드 실패: %s", yaml_file.c_str());
      return;
    }

    std::string image_file = map_yaml["image"].as<std::string>();

    // 절대 경로로 변환
    std::string image_path = "/home/osc/dev_ws/src/yaml/" + image_file;

    RCLCPP_INFO(this->get_logger(), "맵 이미지 경로: %s", image_path.c_str());

    // 이미지 파일이 존재하는지 확인
    struct stat buffer;
    if (stat(image_path.c_str(), &buffer) != 0) {
      RCLCPP_ERROR(this->get_logger(), "맵 이미지 파일이 존재하지 않음: %s", image_path.c_str());
      return;
    }

    resolution_ = map_yaml["resolution"].as<double>();
    origin_x_ = map_yaml["origin"][0].as<double>();
    origin_y_ = map_yaml["origin"][1].as<double>();

    map_image_ = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (map_image_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "맵 이미지 로드 실패: %s", image_path.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "맵 로드 성공: %s", image_path.c_str());
        RCLCPP_INFO(this->get_logger(), "맵 크기: (%d, %d)", map_image_.cols, map_image_.rows);
    }
  }

  // 서비스에서 플랜 요청을 처리하는 메소드
  void handlePlanRequest(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "플랜 요청 처리 중: 시작 (%.2f, %.2f) 목표 (%.2f, %.2f)",
                request->start.pose.position.x, request->start.pose.position.y,
                request->goal.pose.position.x, request->goal.pose.position.y);
    response->plan = createPlan(request->start, request->goal);
  }

  // 시작 지점에서 목표 지점까지의 경로를 생성하는 메소드
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    RCLCPP_INFO(this->get_logger(), "플랜 생성 중");
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "map";

    if (map_image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "맵 이미지가 비어있습니다. 플랜을 생성할 수 없습니다.");
      return path;
    }

    // 다익스트라 알고리즘을 사용하여 포즈 생성
    std::vector<geometry_msgs::msg::PoseStamped> poses = dijkstra(start, goal);
    path.poses = poses;
    RCLCPP_INFO(this->get_logger(), "플랜 생성 완료: %zu 포즈", poses.size());

    // 경로 퍼블리시
    path_publisher_->publish(path);

    return path;
  }

  // 다익스트라 알고리즘을 사용한 경로 찾기 메소드
  std::vector<geometry_msgs::msg::PoseStamped> dijkstra(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    RCLCPP_INFO(this->get_logger(), "다익스트라 알고리즘 시작");
    std::vector<geometry_msgs::msg::PoseStamped> path;
    int width = map_image_.cols;
    int height = map_image_.rows;

    RCLCPP_INFO(this->get_logger(), "맵 크기: 너비=%d, 높이=%d", width, height);

    if (width == 0 || height == 0) {
      RCLCPP_ERROR(this->get_logger(), "잘못된 맵 크기. 다익스트라 알고리즘을 실행할 수 없습니다.");
      return path;
    }

    unsigned int start_x = (start.pose.position.x - origin_x_) / resolution_;
    unsigned int start_y = (start.pose.position.y - origin_y_) / resolution_;
    unsigned int goal_x = (goal.pose.position.x - origin_x_) / resolution_;
    unsigned int goal_y = (goal.pose.position.y - origin_y_) / resolution_;

    RCLCPP_INFO(this->get_logger(), "시작 좌표: x=%.2f, y=%.2f", start.pose.position.x, start.pose.position.y);
    RCLCPP_INFO(this->get_logger(), "목표 좌표: x=%.2f, y=%.2f", goal.pose.position.x, goal.pose.position.y);
    RCLCPP_INFO(this->get_logger(), "시작 맵 좌표: x=%u, y=%u", start_x, start_y);
    RCLCPP_INFO(this->get_logger(), "목표 맵 좌표: x=%u, y=%u", goal_x, goal_y);

    // 열린 리스트를 위한 우선순위 큐
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
    std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));
    std::unordered_map<int, Node> all_nodes;
    std::unordered_map<int, int> came_from;
    
    // 시작 노드 초기화
    open_list.emplace(start_x, start_y, 0.0);
    came_from[start_x + start_y * width] = start_x + start_y * width;

    int step_counter = 0;
    const int visualization_step = 1000;

    // 다익스트라 알고리즘 메인 루프
    while (!open_list.empty()) {
      Node current = open_list.top();
      open_list.pop();

      if (visited[current.x][current.y]) continue;
      visited[current.x][current.y] = true;

      if (current.x == goal_x && current.y == goal_y) {
        break;
      }

      // 4방향 연결된 그리드를 위한 이웃
      std::vector<std::pair<int, int>> neighbors = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0}
      };

      for (auto& offset : neighbors) {
        int new_x = current.x + offset.first;
        int new_y = current.y + offset.second;

        if (new_x < 0 || new_y < 0 || new_x >= width || new_y >= height) continue;

        // 이동 비용
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
        RCLCPP_INFO(this->get_logger(), "방문한 노드: %d, 열린 리스트 크기: %zu", step_counter, open_list.size());
      }
      step_counter++;
    }

    if (came_from.find(goal_x + goal_y * width) == came_from.end()) {
      RCLCPP_ERROR(this->get_logger(), "목표 지점까지의 경로를 찾지 못했습니다.");
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

      // 방향 계산
      if (!path.empty()) {
        double dx = pose.pose.position.x - path.back().pose.position.x;
        double dy = pose.pose.position.y - path.back().pose.position.y;
        pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), std::atan2(dy, dx)));
      }

      path.push_back(pose);
      current = came_from[current];
    }

    std::reverse(path.begin(), path.end());
    RCLCPP_INFO(this->get_logger(), "다익스트라 알고리즘 완료");

    return path;
  }

};

// 메인 함수
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("global_planner"), "MyGlobalPlanner 노드 시작");
  auto node = std::make_shared<MyGlobalPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
