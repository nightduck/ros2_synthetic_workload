#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int64.hpp"
#include "synthetic_workload_msgs/srv/scramble.hpp"

#define SEED1   0xDEADBEEF
#define SEED2   0x8BADF00D
#define SEED3   0xFEEDBABE
#define SEED64  0xFEEDFACECAFEBEEF

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

// Just a garbage operation to kill time
int64_t burn(int64_t in) {
    int32_t lo = (int32_t)in;
    int32_t me = (int32_t)(in >> 16);
    int32_t hi = (int32_t)(in >> 32);

    in = (SEED1 * lo + SEED2 * me + SEED3 * hi) ^ SEED64;
    in ^= in << 1;
    double decimal = (double)in / SEED1;
    return (int64_t)std::floor(decimal * decimal * decimal);
}

class FullNode: public rclcpp::Node
{
public:
    void publishing_timer() {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < 100ms) {
            count_ = burn(count_);
        }

        std_msgs::msg::Int64 msg;
        msg.data = count_;
        la_pub_->publish(msg);
        ma_pub_->publish(msg);
        ha_pub_->publish(msg);
    }

    void high_prio_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < 100ms) {
            msg->data = burn(msg->data);
        }
    }

    void med_prio_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < 100ms) {
            msg->data = burn(msg->data);
        }
    }

    void low_prio_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < 100ms) {
            msg->data = burn(msg->data);
        }
    }

    FullNode() : Node("fullnode") {
        count_ = this->now().nanoseconds();

        ha_pub_ = this->create_publisher<std_msgs::msg::Int64>("ha", 1);
        ma_pub_ = this->create_publisher<std_msgs::msg::Int64>("ma", 1);
        la_pub_ = this->create_publisher<std_msgs::msg::Int64>("la", 1);

        ha_sub_ = this->create_subscription<std_msgs::msg::Int64>("ha", 1,
                                        std::bind(&FullNode::high_prio_callback, this, _1));
        ma_sub_ = this->create_subscription<std_msgs::msg::Int64>("ma", 1,
                                        std::bind(&FullNode::med_prio_callback, this, _1));
        la_sub_ = this->create_subscription<std_msgs::msg::Int64>("la", 1,
                                        std::bind(&FullNode::low_prio_callback, this, _1));

        pub_tmr_ = this->create_wall_timer(500ms, std::bind(&FullNode::publishing_timer, this));
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

class TriPubTimer : public rclcpp::Node
{
public:
    void publishing_timer() {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < 100ms) {
            count_ = burn(count_);
        }

        std_msgs::msg::Int64 msg;
        msg.data = count_;
        la_pub_->publish(msg);
        ma_pub_->publish(msg);
        ha_pub_->publish(msg);
    }

    TriPubTimer() : rclcpp::Node("timer") {
        ha_pub_ = this->create_publisher<std_msgs::msg::Int64>("ha", 1);
        ma_pub_ = this->create_publisher<std_msgs::msg::Int64>("ma", 1);
        la_pub_ = this->create_publisher<std_msgs::msg::Int64>("la", 1);
        timer_ = this->create_wall_timer(500ms, std::bind(&TriPubTimer::publishing_timer, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr ha_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr ma_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr la_pub_;
    size_t count_;
};

class Sub : public rclcpp::Node
{
public:
    void subscriber_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < ms_) {
            msg->data = burn(msg->data);
        }
    }

    Sub(std::string sub_topic, std::chrono::milliseconds ms) : rclcpp::Node("sub_" + sub_topic) {
        ms_ = ms;
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>(sub_topic, 10,
                                std::bind(&Sub::subscriber_callback, this, _1));
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    std::chrono::milliseconds ms_;
    size_t count_;
};


class HiSub : public rclcpp::Node
{
public:
    void hi_subscriber_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < ms_) {
            msg->data = burn(msg->data);
        }
    }

    HiSub(std::string sub_topic, std::chrono::milliseconds ms) : rclcpp::Node("hisub") {
        ms_ = ms;
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>(sub_topic, 10,
                                std::bind(&HiSub::hi_subscriber_callback, this, _1));
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    std::chrono::milliseconds ms_;
    size_t count_;
};

class MeSub : public rclcpp::Node
{
public:
    void me_subscriber_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < ms_) {
            msg->data = burn(msg->data);
        }
    }

    MeSub(std::string sub_topic, std::chrono::milliseconds ms) : rclcpp::Node("mesub") {
        ms_ = ms;
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>(sub_topic, 10,
                                std::bind(&MeSub::me_subscriber_callback, this, _1));
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    std::chrono::milliseconds ms_;
    size_t count_;
};

class LoSub : public rclcpp::Node
{
public:
    void lo_subscriber_callback(const std_msgs::msg::Int64::SharedPtr msg) {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < ms_) {
            msg->data = burn(msg->data);
        }
    }

    LoSub(std::string sub_topic, std::chrono::milliseconds ms) : rclcpp::Node("losub") {
        ms_ = ms;
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>(sub_topic, 10,
                                std::bind(&LoSub::lo_subscriber_callback, this, _1));
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    std::chrono::milliseconds ms_;
    size_t count_;
};

class HiServ : public rclcpp::Node
{
public:
    void hi_service_callback(const synthetic_workload_msgs::srv::Scramble::Request::SharedPtr request,
                            synthetic_workload_msgs::srv::Scramble::Response::SharedPtr response) {
        // Burn time here
        rclcpp::Time start = this->now();
        response->output = request->input;
        while(this->now() - start < ms_) {
            response->output = burn(response->output);
        }
    }

    HiServ(std::string topic, std::chrono::milliseconds ms) : rclcpp::Node("hiserv") {
        ms_ = ms;
        service_ = this->create_service<synthetic_workload_msgs::srv::Scramble>(topic,
                            std::bind(&HiServ::hi_service_callback, this, _1, _2));
    }

private:
    rclcpp::Service<synthetic_workload_msgs::srv::Scramble>::SharedPtr service_;
    std::chrono::milliseconds ms_;
    size_t count_;
};

class MeServ : public rclcpp::Node
{
public:
    void me_service_callback(const synthetic_workload_msgs::srv::Scramble::Request::SharedPtr request,
                            synthetic_workload_msgs::srv::Scramble::Response::SharedPtr response) {
        // Burn time here
        rclcpp::Time start = this->now();
        response->output = request->input;
        while(this->now() - start < ms_) {
            response->output = burn(response->output);
        }
    }

    MeServ(std::string topic, std::chrono::milliseconds ms) : rclcpp::Node("meserv") {
        ms_ = ms;
        service_ = this->create_service<synthetic_workload_msgs::srv::Scramble>(topic,
                            std::bind(&MeServ::me_service_callback, this, _1, _2));
    }

private:
    rclcpp::Service<synthetic_workload_msgs::srv::Scramble>::SharedPtr service_;
    std::chrono::milliseconds ms_;
    size_t count_;
};

class LoServ : public rclcpp::Node
{
public:
    void lo_service_callback(const synthetic_workload_msgs::srv::Scramble::Request::SharedPtr request,
                            synthetic_workload_msgs::srv::Scramble::Response::SharedPtr response) {
        // Burn time here
        rclcpp::Time start = this->now();
        response->output = request->input;
        while(this->now() - start < ms_) {
            response->output = burn(response->output);
        }
    }

    LoServ(std::string topic, std::chrono::milliseconds ms) : rclcpp::Node("loserv") {
        ms_ = ms;
        service_ = this->create_service<synthetic_workload_msgs::srv::Scramble>(topic,
                            std::bind(&LoServ::lo_service_callback, this, _1, _2));
    }

private:
    rclcpp::Service<synthetic_workload_msgs::srv::Scramble>::SharedPtr service_;
    std::chrono::milliseconds ms_;
    size_t count_;
};

class Timer1 : public rclcpp::Node
{
public:
    void timer1_callback() {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < ms_) {
            count_ = burn(count_);
        }
    }

    Timer1(std::chrono::milliseconds duration, std::chrono::milliseconds period) : rclcpp::Node("timer1") {
        ms_ = duration;
        timer_ = this->create_wall_timer(period, std::bind(&Timer1::timer1_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds ms_;
    size_t count_;
};

class Timer2 : public rclcpp::Node
{
public:
    void timer2_callback() {
        // Burn time here
        rclcpp::Time start = this->now();
        while(this->now() - start < ms_) {
            count_ = burn(count_);
        }
    }

    Timer2(std::chrono::milliseconds duration, std::chrono::milliseconds period) : rclcpp::Node("timer2") {
        ms_ = duration;
        timer_ = this->create_wall_timer(period, std::bind(&Timer2::timer2_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds ms_;
    size_t count_;
};