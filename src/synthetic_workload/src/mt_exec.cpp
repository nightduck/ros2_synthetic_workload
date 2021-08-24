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

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);

    //auto t1 = std::make_shared<Timer1>("c1a", 100ms);
    //auto s1 = std::make_shared<SubHi1>("c1a", "c1b", 50ms);
    auto fn = std::make_shared<FullNode>();

    exec.add_node(fn);

    exec.spin();

    rclcpp::shutdown();

    return 0;
}
