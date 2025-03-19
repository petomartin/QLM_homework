#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"  // A megfelelő üzenet típust importáljuk

class DivisionNode : public rclcpp::Node
{
public:
    DivisionNode() : Node("division_nodeB")
    {
        // Create subscription to listen to "/input_numbersB" topic
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/input_numbersB", 10,
            std::bind(&DivisionNode::input_callback, this, std::placeholders::_1));

        // Create publisher to send results to "/division_resultB"
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/division_resultB", 10);

        RCLCPP_INFO(this->get_logger(), "Division node started. Waiting for numbers...");
    }

private:
    void input_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Check if exactly two numbers are provided
        if (msg->data.size() != 2)
        {
        RCLCPP_ERROR(this->get_logger(), "Error: Exactly two numbers are required for division.");
        return;
        }

        double numerator = msg->data[0];  
        double denominator = msg->data[1]; 

        if (denominator == 0.0) // Check for division by zero
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Division by zero!");
        }
        else
        {
            // Perform division and publish the result
            std_msgs::msg::Float64MultiArray result_msg;
            result_msg.data.push_back(numerator / denominator); 
            publisher_->publish(result_msg);
            RCLCPP_INFO(this->get_logger(), "Published division result: %f", result_msg.data[0]);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DivisionNode>());
    rclcpp::shutdown();
    return 0;
}
