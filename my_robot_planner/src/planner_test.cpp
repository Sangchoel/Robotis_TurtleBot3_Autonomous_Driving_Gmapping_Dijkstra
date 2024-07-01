#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

class PlannerTest : public rclcpp::Node
{
public:
  PlannerTest() : Node("planner_test")
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&PlannerTest::imuCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&PlannerTest::odomCallback, this, std::placeholders::_1));
    client_ = this->create_client<nav_msgs::srv::GetPlan>("global_planner/create_plan");

    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for global_planner service...");
    }

    RCLCPP_INFO(this->get_logger(), "PlannerTest node has started.");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    current_orientation_ = msg->orientation;
//   RCLCPP_INFO(this->get_logger(), "IMU Orientation: x=%f, y=%f, z=%f, w=%f",
 //               msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_position_ = msg->pose.pose.position;
 //   RCLCPP_INFO(this->get_logger(), "Odometry Position: x=%f, y=%f, z=%f",
  //              msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();

    // 시작점 설정 (Odometry 데이터를 사용)
    request->start.header.frame_id = "map";
    request->start.pose.position = current_position_;
    request->start.pose.orientation = current_orientation_;

    // 목표점 설정 (임의의 목표점)
    request->goal.header.frame_id = "map";
    request->goal.pose.position.x = 1.0;
    request->goal.pose.position.y = 1.0;
    request->goal.pose.orientation.w = 1.0;

    auto result = client_->async_send_request(request, std::bind(&PlannerTest::handlePlanResponse, this, std::placeholders::_1));
  }

  void handlePlanResponse(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture result)
  {
    if (result.get()->plan.poses.size() > 0) {
      RCLCPP_INFO(this->get_logger(), "Path received with %zu poses", result.get()->plan.poses.size());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service global_planner/create_plan");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr client_;
  
  geometry_msgs::msg::Point current_position_;
  geometry_msgs::msg::Quaternion current_orientation_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
