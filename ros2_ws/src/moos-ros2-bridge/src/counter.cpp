#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class CounterNode : public rclcpp::Node{
public:
    CounterNode() : Node("counter_node"), count_(0){
        pub_ = this->create_publisher<std_msgs::msg::Float64>("CounterFromROS", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CounterNode::timerCallback, this)
        );
    }

private:
    void timerCallback(){
        auto msg = std_msgs::msg::Float64();
        msg.data = count_;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", msg.data);
        pub_->publish(msg);
        ++count_;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CounterNode>());
    rclcpp::shutdown();
    return 0;
}