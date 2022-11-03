#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "bounded_message/msg/test_data4m.hpp"

using namespace std::chrono_literals;

class LoanedMessageTalker : public rclcpp::Node
{
public:
    LoanedMessageTalker()
        : Node("loaned_message_talker")
    {
        auto publish_message =
            [this]() -> void
        {
            auto msg = pod_pub_->borrow_loaned_message();
            msg.get().data_array = {0};
            if (count_% size_t(1e3 * 60) == 0)
                RCLCPP_INFO(this->get_logger(), "Publishing count: '%ld'", count_);
            pod_pub_->publish(std::move(msg));
            count_++;
        };

        // Create a publisher with a custom Quality of Service profile.
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        pod_pub_ = this->create_publisher<bounded_message::msg::TestData4m>("chatter", qos);

        // Use a timer to schedule periodic message publishing.
        timer_ = this->create_wall_timer(1ms, publish_message);
    }

private:
    size_t count_ = 1;
    rclcpp::Publisher<bounded_message::msg::TestData4m>::SharedPtr pod_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LoanedMessageTalker>());
    rclcpp::shutdown();

    return 0;
}