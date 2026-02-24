#include "rclcpp/rclcpp.hpp"

class MyFirstNode : public rclcpp::Node
{
public:
  MyFirstNode() : Node("cpp_test"), counter_(0)
  {
    RCLCPP_INFO(this->get_logger(), "MyFirstNode has been started.");
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     std::bind(&MyFirstNode::timer_callback, this));
  }
private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Timer callback triggered: %d", counter_++);
    counter_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  int counter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyFirstNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}