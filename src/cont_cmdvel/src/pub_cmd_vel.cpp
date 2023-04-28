// Copyright 2016 Open Source Robotics Foundation, Inc.
//

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"   //cmd_velのtwist型のヘッダファイルインクルード         
   
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Pub_cmd_vel : public rclcpp::Node               //Pub_cmd_velのクラス宣言
{
public:
  Pub_cmd_vel()                                      //Constructor 
    : Node("cmdvel_node"), count_(0)                //Node name 
  {
    //topic名 "cmd_vel"でque size 10byteのpublisherを作成
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    //timerがPub_cmd_vel class内のtimer_callbackを0.5sごとに呼び出す
    timer_ = this->create_wall_timer(
      500ms, std::bind(&Pub_cmd_vel::timer_callback, this));
  }

private:
  
  void timer_callback()     //0.5sごとに呼び出され、messageを配信する.
  {
   // 配信するTwist messageを用意  
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.3;          //前進速度 走る
    message.angular.z = 0.;         //旋回速度
    //コンソールに出力
    RCLCPP_INFO(this->get_logger(), "Publishing: vd='%f' dphid='%f'", message.linear.x, message.angular.z);
    //messageを配信 
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pub_cmd_vel>());
  rclcpp::shutdown();
  return 0;
}
