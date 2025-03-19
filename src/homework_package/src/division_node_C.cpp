#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class DivisionNode : public rclcpp::Node
{
public:
    DivisionNode() : Node("division_node_C"), numerator_(0.0), denominator_(1.0)
    {
        // Subscribe to the numerator and denominator topics
        numerator_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/input_numbers/numerator", 10, 
            std::bind(&DivisionNode::numerator_callback, this, std::placeholders::_1));

        denominator_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/input_numbers/denominator", 10, 
            std::bind(&DivisionNode::denominator_callback, this, std::placeholders::_1));

        // Publisher to output the division result
        result_pub_ = this->create_publisher<std_msgs::msg::Float64>("/division_resultC", 10);

        RCLCPP_INFO(this->get_logger(), "Division Node has been started.");
    }

private:
    // Callback function for numerator topic
    void numerator_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        numerator_ = msg->data;  
        try_divide();  // Attempt division whenever a new numerator is received
    }

    // Callback function for denominator topic
    void denominator_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        denominator_ = msg->data;  
        try_divide();  // Attempt division whenever a new denominator is received
    }

    // Function to perform the division
    void try_divide()
    {
        // Check for division by zero
        if (denominator_ == 0.0)
        {
            RCLCPP_ERROR(this->get_logger(), "Division by zero error! Cannot divide %f by zero.", numerator_);
            return;
        }

        // Perform the division and publish the result
        auto result_msg = std_msgs::msg::Float64();
        result_msg.data = numerator_ / denominator_;
        result_pub_->publish(result_msg);

        RCLCPP_INFO(this->get_logger(), "Published division result: %f", result_msg.data);
    }

    // Subscription pointers for numerator and denominator
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr numerator_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr denominator_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr result_pub_;

    // Variables to store the numerator and denominator values
    double numerator_;
    double denominator_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<DivisionNode>());  
    rclcpp::shutdown();
    return 0;
}
