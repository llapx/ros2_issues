#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto opt = rclcpp::NodeOptions();
    auto node = rclcpp::Node::make_shared("client_node", opt);
    auto m_client = node->create_client<std_srvs::srv::SetBool>("example_service");
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    while (!m_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            break;
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }
    uint counter = 1;
    do
    {
        RCLCPP_INFO(node->get_logger(), "Sending request: %d", counter);
        auto result = m_client->async_send_request(request);
        RCLCPP_INFO(node->get_logger(), "Waiting for response: %d", counter);
        auto answer = result.get();
        RCLCPP_INFO(node->get_logger(), "Got response: %s", answer->message.c_str());
    } while (++counter < 10000);

    rclcpp::shutdown();

    return 0;
}