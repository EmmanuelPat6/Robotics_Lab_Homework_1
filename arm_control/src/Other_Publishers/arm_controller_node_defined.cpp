// defined_position_publisher.cpp

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <thread>
#include <array>

class DefinedPositionPublisher : public rclcpp::Node
{
public:
    DefinedPositionPublisher() : Node("arm_controller_defined_node")
    {
        // Crea un publisher per il topic /position_controller/commands
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);
    }

    // Funzione pubblica per avviare la pubblicazione dei movimenti predefiniti
    void startPublishing()
    {
        publish_predefined_movements();
    }

private:
    // Funzione privata che pubblica i movimenti predefiniti
    void publish_predefined_movements()
    {
        // Definisci 5 movimenti predefiniti (4 valori ciascuno)
        std::array<std::array<double, 4>, 5> movements = {{
            {0.5, 0.5, 0.5, 0.5},   // Movimento 1
            {0.7, 1.4, 0.5, 0.5},   // Movimento 2
            {0.5, 1.0, 0.6, 0.6},   // Movimento 3
            {0.3, 0.6, 0.3, 0.3},   // Movimento 4
            {0.0, 0.0, 0.0, 0.0}    // Movimento 5
        }};

        for (const auto &movement : movements)
        {
            auto command_msg = std_msgs::msg::Float64MultiArray();
            command_msg.data = {movement[0], movement[1], movement[2], movement[3]};

            // Pubblica il messaggio
            publisher_->publish(command_msg);
            RCLCPP_INFO(this->get_logger(), "Publishing: [%f, %f, %f, %f]",
                         command_msg.data[0], command_msg.data[1], 
                         command_msg.data[2], command_msg.data[3]);

            // Attendi 10 secondi prima della pubblicazione successiva
            std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto position_publisher = std::make_shared<DefinedPositionPublisher>();

    // Avvia la pubblicazione dei movimenti predefiniti
    position_publisher->startPublishing();

    rclcpp::shutdown();
    return 0;
}
