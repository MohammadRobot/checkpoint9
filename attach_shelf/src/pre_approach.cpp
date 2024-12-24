/* 
ROS2 humble

By @MohammadRobot

*/

#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <algorithm>
// #include <cmath>
#include <limits>
#include <math.h>

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/laser_scan.hpp"



using namespace std::chrono_literals;

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach_node") {

    this->declare_parameter("obstacle", 0.0);
    this->declare_parameter("degrees", 0);

    mode = "get_parameters";
    

    pub_cmd_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("diffbot_base_controller/cmd_vel_unstamped", 10);

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

    sub_laserScan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        100ms, std::bind(&PreApproach::timer_callback, this));
  }

  void getting_params() {
    obstacle = this->get_parameter("obstacle").get_parameter_value().get<float>();
    degrees = this->get_parameter("degrees").get_parameter_value().get<int>();
    RCLCPP_INFO(this->get_logger(), "parameters_succeeded");
	mode ="parameters_succeeded";
  }


private:
  std::string mode;
  float obstacle;
  int  degrees;

  float obstacle_distance;

  bool position_reached = false;

  const double LINEAR_TOLERANCE_ = 0.02;  // meters
  const double ANGULAR_TOLERANCE_ = 0.01; // radians
  const double K_LINEAR = 1.5;            // Linear velocity gain
  const double K_ANGULAR = 2.0;           // Angular velocity gain
  const double MAX_LINEAR_SPEED = 0.5;    // m/s
  const double MAX_ANGULAR_SPEED = 1.0;   // rad/s

 // Robot state
  double roll, pitch, yaw;
  geometry_msgs::msg::Pose2D desired_pos_, current_pos_;

  // Utility function to normalize angle to [-pi, pi]
  double normalize_angle(double angle) { return atan2(sin(angle), cos(angle)); }


  void timer_callback() {
	    if (mode == "get_parameters") {
			getting_params();
        } else if (mode == "parameters_succeeded") {
			go_to_pose();
		} else if (mode == "pose_succeeded") {
			adjust_orientation();
        } else if (mode == "turn_succeeded") {
			RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            mode = "goal_succeeded";
		}else{
        
        }
  }


  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f'",
    // msg->pose.pose.position.x);

    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;

    // convert Quaternion to Euler
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    current_pos_.theta = yaw;

    // RCLCPP_INFO(this->get_logger(), "current_pos_.x[%f] current_pos_.y[%f]  current_pos_.theta[%f]",
    //                                 current_pos_.x,current_pos_.y,current_pos_.theta);

  }


  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {

    int laserScanPoints = static_cast<int>(round(((scan_msg->angle_max - scan_msg->angle_min) /
                                scan_msg->angle_increment)));

		//get front laser Scan point 
	int obstacle_angel = 	static_cast<int>(round((laserScanPoints / 2)));
																												
	obstacle_distance = scan_msg->ranges[obstacle_angel];

			// RCLCPP_INFO(this->get_logger(), "obstacle_distance[%f]",
			//                                 obstacle_distance);
  }

  void go_to_pose(){

    desired_pos_.x = current_pos_.x + (obstacle_distance - obstacle ); // 0.21 is the laser to robot base link distaance 
	desired_pos_.y= current_pos_.y;

    // compute the difference between the current position and the desired
		// position
		double diff_x_ = desired_pos_.x - current_pos_.x;
		double diff_y_ = desired_pos_.y - current_pos_.y;
		double distance_error = sqrt(diff_x_ * diff_x_ + diff_y_ * diff_y_);
		// Compute the angle to the goal
		double angle_to_goal = atan2(diff_y_, diff_x_);
		double angle_error = normalize_angle(angle_to_goal - current_pos_.theta);

		// RCLCPP_INFO(this->get_logger(), "distance_error[%f]	angle_error[%f]",
		// 						distance_error, angle_error);

		//Move to the position
		if (distance_error < LINEAR_TOLERANCE_) {
			// Position reached, now switch to orientation adjustment
			stop_robot();
			mode = "pose_succeeded";
			RCLCPP_INFO(this->get_logger(),"Position reached");
			return;
		} else {
			// Control law to move to the position (move forward and adjust
			// heading)
			if (fabs(angle_error) > ANGULAR_TOLERANCE_) {
				// Rotate towards the goal
				twist.linear.x = 0.0;
				twist.angular.z = K_ANGULAR * angle_error;
			} else {
				// Move towards the goal
				twist.linear.x = K_LINEAR * distance_error;
				twist.angular.z = 0.0;
			}
		}
	
		publish_velocity();
  }

void adjust_orientation(){
    desired_pos_.theta = (degrees * (M_PI / 180)); // Convert to Degree to radian
	// Adjust orientation
	double angle_error = normalize_angle(desired_pos_.theta - current_pos_.theta);

    // RCLCPP_INFO(this->get_logger(), "angle_error[%f]", angle_error);
	
	if (fabs(angle_error) < ANGULAR_TOLERANCE_) {
		// Orientation reached, stop the robot
		stop_robot();
		mode = "turn_succeeded";
		RCLCPP_INFO(this->get_logger(), "Orientation succeeded");
		return;
	} else {
		// Rotate in place to adjust the orientation
		twist.linear.x = 0.0;
		twist.angular.z = K_ANGULAR * angle_error;
	}

	publish_velocity();
}

void stop_robot(){
	twist.linear.x = 0.0;
	twist.angular.z = 0.0;
	publish_velocity();
}

void publish_velocity(){

	// Limit velocities
	twist.linear.x = std::min(twist.linear.x, MAX_LINEAR_SPEED);
	twist.angular.z = std::min(std::max(twist.angular.z, -MAX_ANGULAR_SPEED),
														MAX_ANGULAR_SPEED);

	// Publish velocity command
	pub_cmd_vel_->publish(twist);

}

  geometry_msgs::msg::Twist twist;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserScan_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}