#include "rclcpp/rclcpp.hpp"
#include "test_interfaces/srv/test_srv.hpp"

#include <memory>

void test_service(const std::shared_ptr<test_interfaces::srv::TestSrv::Request> request,
          std::shared_ptr<test_interfaces::srv::TestSrv::Response>      response)
{
  response->retval = request->paramval * request->paramval;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nparamval: %ld",
                request->paramval);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->retval);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_server");

  rclcpp::Service<test_interfaces::srv::TestSrv>::SharedPtr service =
    node->create_service<test_interfaces::srv::TestSrv>("test_srv", &test_service);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to serve.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}