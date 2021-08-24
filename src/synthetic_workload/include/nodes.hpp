#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int64.hpp"

#define SEED1   0xDEADBEEF
#define SEED2   0x8BADF00D
#define SEED3   0xFEEDBABE
#define SEED64  0xFEEDFACECAFEBEEF

using namespace std::chrono_literals;
using std::placeholders::_1;

std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

int64_t burn(int64_t in) {
    int32_t lo = (int32_t)in;
    int32_t me = (int32_t)(in >> 16);
    int32_t hi = (int32_t)(in >> 32);

    in = (SEED1 * lo + SEED2 * me + SEED3 * hi) ^ SEED64;

    double decimal = (double)in / SEED1;
    return (int64_t)std::floor(decimal * decimal * decimal);
}

class FullNode: public rclcpp::Node
{
public:
    void empty_timer() {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < 20ms) {
            count_ = burn(count_);
        }

        RCLCPP_INFO(this->get_logger(), "Empty timer");
    }

    void publishing_timer() {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < 10ms) {
            count_ = burn(count_);
        }

        std_msgs::msg::Int64 msg;
        msg.data = count_;
        lb_pub_->publish(msg);
        mb_pub_->publish(msg);
        hb_pub_->publish(msg);
    }

    void high_prio_stage_a_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < 20ms) {
            msg->data = burn(msg->data);
        }

        hb_pub_->publish(*msg);
    }

    void med_prio_stage_a_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < 20ms) {
            msg->data = burn(msg->data);
        }

        mb_pub_->publish(*msg);
    }

    void low_prio_stage_a_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < 20ms) {
            msg->data = burn(msg->data);
        }

        lb_pub_->publish(*msg);
    }

    void high_prio_stage_b_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < 10ms) {
            msg->data = burn(msg->data);
        }

        RCLCPP_INFO(this->get_logger(), "Hi prio end");
    }

    void med_prio_stage_b_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < 10ms) {
            msg->data = burn(msg->data);
        }

        RCLCPP_INFO(this->get_logger(), "Med prio end");
    }

    void low_prio_stage_b_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < 10ms) {
            msg->data = burn(msg->data);
        }

        RCLCPP_INFO(this->get_logger(), "Low prio end");
    }

    FullNode() : Node("fullnode") {
        count_ = this->now().nanoseconds();

        ha_pub_ = this->create_publisher<std_msgs::msg::Int64>("ha", 1);
        ma_pub_ = this->create_publisher<std_msgs::msg::Int64>("ma", 1);
        la_pub_ = this->create_publisher<std_msgs::msg::Int64>("la", 1);
        hb_pub_ = this->create_publisher<std_msgs::msg::Int64>("hb", 1);
        mb_pub_ = this->create_publisher<std_msgs::msg::Int64>("mb", 1);
        lb_pub_ = this->create_publisher<std_msgs::msg::Int64>("lb", 1);

        ha_sub_ = this->create_subscription<std_msgs::msg::Int64>("ha", 1,
                                        std::bind(&FullNode::high_prio_stage_a_callback, this, _1));
        ma_sub_ = this->create_subscription<std_msgs::msg::Int64>("ma", 1,
                                        std::bind(&FullNode::med_prio_stage_a_callback, this, _1));
        la_sub_ = this->create_subscription<std_msgs::msg::Int64>("la", 1,
                                        std::bind(&FullNode::low_prio_stage_a_callback, this, _1));
        hb_sub_ = this->create_subscription<std_msgs::msg::Int64>("hb", 1,
                                        std::bind(&FullNode::high_prio_stage_b_callback, this, _1));
        mb_sub_ = this->create_subscription<std_msgs::msg::Int64>("mb", 1,
                                        std::bind(&FullNode::med_prio_stage_b_callback, this, _1));
        lb_sub_ = this->create_subscription<std_msgs::msg::Int64>("lb", 1,
                                        std::bind(&FullNode::low_prio_stage_b_callback, this, _1));

        pub_tmr_ = this->create_wall_timer(150ms, std::bind(&FullNode::publishing_timer, this));
        donothing_tmr_ = this->create_wall_timer(75ms, std::bind(&FullNode::empty_timer, this));
    }

private:
    int64_t count_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr ha_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr ma_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr la_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr hb_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr mb_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr lb_pub_;

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ha_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ma_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr la_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr hb_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr mb_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr lb_sub_;

    rclcpp::TimerBase::SharedPtr pub_tmr_;
    rclcpp::TimerBase::SharedPtr donothing_tmr_;
};

class Timer1 : public rclcpp::Node
{
public:
    void timer1_callback() {
        rclcpp::Time now = this->now();
        RCLCPP_INFO(this->get_logger(), "Callback start: %.9f", now.seconds());

        auto message = std_msgs::msg::Int64();
        message.data = this->count_++;

        // Burn time here
        for(int i = 0; i < 1000000; i++) {
            message.data = burn(message.data);
        }

        // // Publish current thread
        // auto curr_thread = string_thread_id();
        // RCLCPP_INFO(
        // this->get_logger(), "\n<<THREAD %s>> Publishing '%d'",
        // curr_thread.c_str(), message.data);

        this->publisher_->publish(message);
        
        now = this->now();
        RCLCPP_INFO(this->get_logger(), "Callback end:   %.9f", now.seconds());
    };

    Timer1(std::string topic, std::chrono::milliseconds ms) : rclcpp::Node("timer") {
        publisher_ = this->create_publisher<std_msgs::msg::Int64>(topic, 1);
        timer_ = this->create_wall_timer(ms, std::bind(&Timer1::timer1_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    size_t count_;
};


class SubHi1 : public rclcpp::Node
{
public:
    void subpub_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        for(int i = 0; i < 10000; i++) {
            msg->data = burn(msg->data);
        }

        this->publisher_->publish(*msg);
    };

    SubHi1(std::string sub_topic, std::string pub_topic, std::chrono::milliseconds ms) : rclcpp::Node("sh1") {
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>(sub_topic, 1,
                                std::bind(&SubHi1::subpub_callback, this, _1));
        publisher_ = this->create_publisher<std_msgs::msg::Int64>(pub_topic, 1);
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    size_t count_;
};