#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "test_interfaces/msg/test_msg.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<test_interfaces::msg::TestMsg>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const test_interfaces::msg::TestMsg & msg) const
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.testval << "'");
  }
  rclcpp::Subscription<test_interfaces::msg::TestMsg>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}