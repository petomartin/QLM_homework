#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class DivisionNode : public rclcpp::Node
{
public:
    DivisionNode() : Node("division_node"), numerator_(0.0), denominator_(1.0), received_numerator_(false), received_denominator_(false)
    {
        // Set up the subscription that waits for the numbers
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/input_numbers", 10,
            std::bind(&DivisionNode::input_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/division_result", 10);

        RCLCPP_INFO(this->get_logger(), "Division node started. Waiting for numbers...");
    }

private:
    void input_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        if (!received_numerator_)
        {
            numerator_ = msg->data;
            received_numerator_ = true;
            RCLCPP_INFO(this->get_logger(), "Received numerator: %f", numerator_);
        }
        else if (!received_denominator_)
        {
            denominator_ = msg->data;
            received_denominator_ = true;
            RCLCPP_INFO(this->get_logger(), "Received denominator: %f", denominator_);

            if (denominator_ == 0.0)  // If the denominator is 0, report division by zero error
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Division by zero!");
            }
            else
            {
                // If no division by zero, perform the division and publish the result
                std_msgs::msg::Float64 result_msg;
                result_msg.data = numerator_ / denominator_;
                publisher_->publish(result_msg);
                RCLCPP_INFO(this->get_logger(), "Published division result: %f", result_msg.data);
            }

            // After performing the division, reset the flags to accept new numbers
            received_numerator_ = false;
            received_denominator_ = false;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    
    // Variables that store the numerator, denominator, and their reception status
    double numerator_;
    double denominator_;
    bool received_numerator_;
    bool received_denominator_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DivisionNode>());
    rclcpp::shutdown();
    return 0;
}