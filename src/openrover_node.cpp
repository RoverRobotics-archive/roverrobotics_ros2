#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;



class OpenRoverNode : public rclcpp::Node {
public:
  OpenRoverNode();


protected:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};


OpenRoverNode::OpenRoverNode() : Node("openrover_core"){
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic");
        timer_ = this->create_wall_timer(500ms, std::bind(&OpenRoverNode::timer_callback, this));
};

void OpenRoverNode::timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world!";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  };

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenRoverNode>());
  rclcpp::shutdown();
  return 0;
}