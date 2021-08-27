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
            std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);

    // Declaring nodes for each callback
    auto flood1 = std::make_shared<Sub>("flood", 500ms);     // Not tracking this, just call it to flood the executor with work and control when the polling point occurs
    auto flood2 = std::make_shared<Sub>("flood", 520ms);
    auto sh = std::make_shared<HiServ>("hiserv", 100ms);
    auto sm = std::make_shared<MeServ>("meserv", 100ms);
    auto sl = std::make_shared<LoServ>("loserv", 100ms);
    auto h = std::make_shared<HiSub>("hisub", 100ms);
    auto m = std::make_shared<MeSub>("mesub", 100ms);
    auto l = std::make_shared<LoSub>("losub", 100ms);
    Timer1::SharedPtr t1, t2;

    // Declare publishers and clients to respond to them
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pub_crawl");
    auto flood_pub = node->create_publisher<std_msgs::msg::Int64>("flood", 10);
    auto sh_client = node->create_client<synthetic_workload_msgs::srv::Scramble>("hiserv");
    auto sm_client = node->create_client<synthetic_workload_msgs::srv::Scramble>("meserv");
    auto sl_client = node->create_client<synthetic_workload_msgs::srv::Scramble>("loserv");
    auto h_pub = node->create_publisher<std_msgs::msg::Int64>("hisub", 10);
    auto m_pub = node->create_publisher<std_msgs::msg::Int64>("mesub", 10);
    auto l_pub = node->create_publisher<std_msgs::msg::Int64>("losub", 10);

    exec->add_node(flood1);
    exec->add_node(flood2);
    exec->add_node(sh);
    exec->add_node(sm);
    exec->add_node(sl);
    exec->add_node(h);
    exec->add_node(m);
    exec->add_node(l);


    // Make other thread to publish stuff
    std::thread thr([&](){
        auto request = std::make_shared<synthetic_workload_msgs::srv::Scramble::Request>();
        request->input = node->now().nanoseconds();

        auto msg = std::make_shared<std_msgs::msg::Int64>();
        msg->data = node->now().nanoseconds();

        // Flood all executor threads
        flood_pub->publish(*msg);

        rclcpp::sleep_for(50ms);

        // Sleep and create timers 50ms before flood finishes (so they trigger 50ms after)
        t1 = std::make_shared<Timer1>(100ms, 500ms);
        t2 = std::make_shared<Timer2>(100ms, 500ms);
        exec->add_node(t1);
        exec->add_node(t2);
        RCLCPP_INFO(node->get_logger(), "Assigned timers");

        while (!sh_client->wait_for_service(1s) || !sm_client->wait_for_service(1s) || !sl_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        // Publish initial L M H SH SL L M H SH SL sequence
        l_pub->publish(*msg);
        m_pub->publish(*msg);
        h_pub->publish(*msg);
        sl_client->async_send_request(request);
        sh_client->async_send_request(request);
        l_pub->publish(*msg);
        m_pub->publish(*msg);
        h_pub->publish(*msg);
        sl_client->async_send_request(request);
        sh_client->async_send_request(request);

        RCLCPP_INFO(node->get_logger(), "Published L M H SH SL L M H SH SL");

        // Wait 150ms after flood and send SM SM L sequence
        rclcpp::sleep_for(600ms);
        sm_client->async_send_request(request);
        sm_client->async_send_request(request);
        l_pub->publish(*msg);

        RCLCPP_INFO(node->get_logger(), "Published SM SM H");

        // Wait 650ms after flood and send M H
        rclcpp::sleep_for(500ms);
        m_pub->publish(*msg);
        h_pub->publish(*msg);

        RCLCPP_INFO(node->get_logger(), "Published L M");

        return 0;
    });

    exec->spin();

    RCLCPP_INFO(node->get_logger(), "Executor finished");

    rclcpp::shutdown();

    return 0;
}
