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

    auto tmr = std::make_shared<StaggeredPubTimer>();
    auto hi = std::make_shared<Sub>("ha", 80ms);
    auto me = std::make_shared<Sub>("ma", 80ms);
    auto lo = std::make_shared<Sub>("la", 80ms);

    exec.add_node(tmr);
    exec.add_node(hi);
    exec.add_node(me);
    exec.add_node(lo);

    exec.spin();

    rclcpp::shutdown();

    return 0;
}
