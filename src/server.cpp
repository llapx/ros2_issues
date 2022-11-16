#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class ServerNode : public rclcpp::Node
{
public:
    ServerNode(const rclcpp::NodeOptions &options)
        : Node("server_node", options)
    {
        auto handle_request =
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
        {
            static unsigned int serial_num = 1;
            (void)request;
            RCLCPP_INFO(this->get_logger(), "Received service client request... Sending response: %d", serial_num);
            response->success = true;
            response->message = std::to_string(serial_num++);
        };
        srv_ = create_service<std_srvs::srv::SetBool>("example_service", handle_request);
    }

private:
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto opt = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<ServerNode>(opt));
    rclcpp::shutdown();

    return 0;
}