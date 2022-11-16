#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;

class TestParams : public rclcpp::Node
{
public:
    TestParams(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("this_node", options)
    {
#if 0
        auto declare_parameters_results = this->declare_parameters(
            "hello",
            {
                rclcpp::Parameter("foo", 2),
                rclcpp::Parameter("bar", "hello"),
                rclcpp::Parameter("baz", 1.45),
                rclcpp::Parameter("foobar", true),
            });
#else
        rcl_interfaces::msg::ParameterDescriptor foo_desc;
        foo_desc.description = "Foo";
        rcl_interfaces::msg::ParameterDescriptor bar_desc;
        bar_desc.description = "Bar";
        auto declare_parameters_results = this->declare_parameters(
            "",
            {
                {rclcpp::Parameter("", 2), foo_desc},
                {rclcpp::Parameter("bar", "hello"), bar_desc},
            });
#endif
    }
private:
};

// Code below is just to start the node
int main(int argc, char **argv)
{
    std::map<rclcpp::Parameter, rcl_interfaces::msg::ParameterDescriptor> m;
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options = rclcpp::NodeOptions();
    auto node = std::make_shared<TestParams>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}