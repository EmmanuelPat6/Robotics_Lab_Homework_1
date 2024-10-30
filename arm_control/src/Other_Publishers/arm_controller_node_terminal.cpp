// position_publisher
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>

class PositionPublisher : public rclcpp::Node
{
public:
    PositionPublisher() : Node("arm_controller_node")
    {
        // Publisher for the topic /position_controller/commands
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);
    }

    // Public method to start the publishing process
    void startPublishing()
    {
        // Call the private method for publishing messages
        publish_messages();
    }

private:
    // Private method to handle the publishing logic
    void publish_messages()
    {
        while (rclcpp::ok())
        {
            // Joint Variables
            std::cout << "Enter the 4 Joint Variables to send (separated by spaces): ";
            std::string input;
            std::getline(std::cin, input);

            // Message Type = Float64MultiArray
            auto command_msg = std_msgs::msg::Float64MultiArray();
            std::istringstream iss(input);
            double value;

            // Read up to 4 values from the input string
            int count = 0;
            while (iss >> value && count < 4)
            {
                command_msg.data.push_back(value); // Add the value to the message
                count++;
            }

            // Check if exactly 4 values were entered
            if (count == 4)
            {
                // Publish the message
                publisher_->publish(command_msg);
                RCLCPP_INFO(this->get_logger(), "Publishing: [%f, %f, %f, %f]",
                             command_msg.data[0], command_msg.data[1], 
                             command_msg.data[2], command_msg.data[3]);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Please enter exactly 4 numbers.");
            }
        }
    }

    // Declare the pointer for the publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto position_publisher = std::make_shared<PositionPublisher>();

    // Use the public method to start publishing
    position_publisher->startPublishing();

    rclcpp::shutdown();
    return 0;
}
