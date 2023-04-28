// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"       //odometry型のヘッダファイルインクルード

using std::placeholders::_1;

class Sub_odom : public rclcpp::Node       //Sub_odomのクラス宣言
{
public:
  Sub_odom()                               //Constructor 
  : Node("odom_node")                   //Node name (cont_cmd_velと同じnodeにしてみる)
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&Sub_odom::topic_callback, this, _1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
   {
    RCLCPP_INFO(this->get_logger(), 
    "Subcribing: x='%f' y='%f' z='%f' ori='%f %f %f %f' linx='%f' anular='%f' ",
     msg->pose.pose.position.x, 
     msg->pose.pose.position.y,
     msg->pose.pose.position.z,
     msg->pose.pose.orientation.x,
     msg->pose.pose.orientation.y,
     msg->pose.pose.orientation.z,
     msg->pose.pose.orientation.w,
     msg->twist.twist.linear.x,
     msg->twist.twist.angular.z);
    
   }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sub_odom>());
  rclcpp::shutdown();
  return 0;
}
