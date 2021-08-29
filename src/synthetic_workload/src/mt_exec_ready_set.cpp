#include "nodes.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    // Measure time to chew cycles on this machine
    {
        RCLCPP_INFO(rclcpp::get_logger("global_logger"), "Benchmarking...");
        auto clock = rclcpp::Clock();
        rclcpp::Time start = clock.now();

        uint64_t x = start.nanoseconds();

        // Burn time here
        for(int i = 0; i < 100000000; i++) {
            x = burn(x);
        }
        
        rclcpp::Duration duration = clock.now() - start;
        RCLCPP_INFO(rclcpp::get_logger("global_logger"), "This machine runs burn in %.9fns, result %lld", 10*duration.seconds(), x);
    }

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);

    auto hi = std::make_shared<HiSub>("h", 100ms);
    auto me = std::make_shared<MeSub>("m", 150ms);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pub_crawl");
    auto h_pub = node->create_publisher<std_msgs::msg::Int64>("h", 10);
    auto m_pub = node->create_publisher<std_msgs::msg::Int64>("m", 10);

    exec.add_node(hi);
    exec.add_node(me);

    auto msg = std::make_shared<std_msgs::msg::Int64>();
    msg->data = node->now().nanoseconds();
    h_pub->publish(*msg);
    m_pub->publish(*msg);
    m_pub->publish(*msg);

    RCLCPP_INFO(node->get_logger(), "Published M M H");

    exec.spin();

    rclcpp::shutdown();

    return 0;
}
