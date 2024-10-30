#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
using namespace std::chrono_literals;

class ArmController : public rclcpp::Node {
public:
    ArmController() : Node("arm_controller") {
        // Subscriber for the topic /joint_states
        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ArmController::jointStateCallback, this, std::placeholders::_1));

        // Publisher for the topic /position_controller/commands
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);
        timer_ = this->create_wall_timer(
             500ms, std::bind(&ArmController::jointCommand, this));

    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Joint State Values
        RCLCPP_INFO(this->get_logger(), "Joint States:");
        for (size_t i = 0; i < msg->name.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "%s Position: %f", msg->name[i].c_str(), msg->position[i]);
        }
    }
    void jointCommand() {
        auto command_msg = std_msgs::msg::Float64MultiArray();
        command_msg.data = {1.0, 0.5, 0.1, 0.5};

        // Publish the command
        publisher_->publish(command_msg);
        //RCLCPP_INFO(this->get_logger(), "Publishing command: [%f, %f, %f, %f]",
          //           command_msg.data[0], command_msg.data[1], command_msg.data[2], command_msg.data[3]);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmController>());

//    auto position_publisher = std::make_shared<ArmController>();
  //  position_publisher->jointCommand();

    rclcpp::shutdown();
    return 0;
}

