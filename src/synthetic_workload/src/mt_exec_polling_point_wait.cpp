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

    rclcpp::executors::MultiThreadedExecutor::SharedPtr exec =
            std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);

    // Declaring nodes for each callback
    auto flood1 = std::make_shared<Sub>("flood", 20ms);     // Not tracking this, just call it to flood the executor with work and control when the polling point occurs
    auto flood2 = std::make_shared<Sub>("flood", 40ms);
    auto flood3 = std::make_shared<Sub>("flood", 60ms);
    auto flood4 = std::make_shared<Sub>("flood", 80ms);
    auto s = std::make_shared<Sub>("sub", 10ms);

    // Declare publishers and clients to respond to them
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pub_crawl");
    auto flood_pub = node->create_publisher<std_msgs::msg::Int64>("flood", 10);
    auto s_pub = node->create_publisher<std_msgs::msg::Int64>("sub", 10);

    exec->add_node(flood1);
    exec->add_node(flood2);
    exec->add_node(flood3);
    exec->add_node(flood4);
    exec->add_node(s);


    // Make other thread to publish stuff
    std::thread thr([&](){
        auto msg = std::make_shared<std_msgs::msg::Int64>();
        msg->data = node->now().nanoseconds();

        // Flood all executor threads
        flood_pub->publish(*msg);

        rclcpp::sleep_for(100ms);

        for(int i = 0; i < 20; i++) {
            s_pub->publish(*msg);
            rclcpp::sleep_for(20ms);
        }

        RCLCPP_INFO(node->get_logger(), "Finished (press Ctrl+C)");

        return 0;
    });

    exec->spin();

    RCLCPP_INFO(node->get_logger(), "Executor finished");

    rclcpp::shutdown();

    return 0;
}
