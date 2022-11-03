#include "rclcpp/rclcpp.hpp"
#include "bounded_message/msg/test_data4m.hpp"

class Listener : public rclcpp::Node
{
public:
    Listener()
        : Node("listener")
    {
        auto callback =
            [this](bounded_message::msg::TestData4m::ConstSharedPtr msg) -> void
        {
            if (count_ % size_t(1e3) == 0)
                RCLCPP_INFO(this->get_logger(), "I heard %ld times data with size of [%ld]", count_, msg->data_array.size());
            count_++;
        };
        sub_ = create_subscription<bounded_message::msg::TestData4m>("chatter", 1, callback);
    }

private:
    size_t count_ = 0;
    rclcpp::Subscription<bounded_message::msg::TestData4m>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();

    return 0;
}