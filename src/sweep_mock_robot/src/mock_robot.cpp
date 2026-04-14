#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cstdio>

class MockRobotNode : public rclcpp::Node
{
public:
    MockRobotNode()
        : Node("mock_robot_node"),
          current_status_("待机"),
          x_(0.0), y_(0.0), yaw_(0.0),
          linear_x_(0.0), angular_z_(0.0),
          battery_(100)
    {
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/agv/status", 10);

        telemetry_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/agv/telemetry", 10);

        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/ui/task_cmd", 10,
            std::bind(&MockRobotNode::onTaskCommand, this, std::placeholders::_1));

        params_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/ui/process_params", 10,
            std::bind(&MockRobotNode::onProcessParams, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MockRobotNode::onCmdVel, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MockRobotNode::onTimer, this));

        RCLCPP_INFO(this->get_logger(), "mock_robot_node started.");
    }

private:
    void onTaskCommand(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string &cmd = msg->data;

        if (cmd == "start") {
            current_status_ = "作业中";
            linear_x_ = 0.4;
            angular_z_ = 0.0;
        } else if (cmd == "pause") {
            current_status_ = "已暂停";
            linear_x_ = 0.0;
            angular_z_ = 0.0;
        } else if (cmd == "stop") {
            current_status_ = "已停止";
            linear_x_ = 0.0;
            angular_z_ = 0.0;
        } else if (cmd == "charge") {
            current_status_ = "回充中";
            linear_x_ = 0.2;
            angular_z_ = 0.0;
        } else {
            current_status_ = "未知命令";
        }

        RCLCPP_INFO(this->get_logger(), "received task cmd: %s", cmd.c_str());
        publishStatus();
        publishTelemetry();
    }

    void onProcessParams(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "received process params: %s", msg->data.c_str());
        current_status_ = "参数已更新";
        publishStatus();
        publishTelemetry();
    }

    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        linear_x_ = msg->linear.x;
        angular_z_ = msg->angular.z;

        if (linear_x_ > 0.0) {
            current_status_ = "手动前进";
        } else if (linear_x_ < 0.0) {
            current_status_ = "手动后退";
        } else if (angular_z_ > 0.0) {
            current_status_ = "手动左转";
        } else if (angular_z_ < 0.0) {
            current_status_ = "手动右转";
        } else {
            current_status_ = "手动停止";
        }

        RCLCPP_INFO(this->get_logger(),
                    "received cmd_vel: linear.x=%.2f angular.z=%.2f",
                    linear_x_, angular_z_);

        publishStatus();
        publishTelemetry();
    }

    void onTimer()
    {
        x_ += linear_x_ * 1.0;
        y_ += 0.1 * angular_z_;
        yaw_ += angular_z_ * 0.2;

        if (battery_ > 0) {
            if (current_status_ == "作业中" || current_status_.rfind("手动", 0) == 0) {
                battery_ -= 1;
            } else if (current_status_ == "回充中" && battery_ < 100) {
                battery_ += 2;
            }
        } 

        if (battery_ > 100) battery_ = 100;
        if (battery_ < 0) battery_ = 0;

        publishStatus();
        publishTelemetry();
    }

    void publishStatus()
    {
        std_msgs::msg::String msg;
        msg.data = current_status_;
        status_pub_->publish(msg);
    }

    void publishTelemetry()
    {
        std_msgs::msg::String msg;
        msg.data =
            "x=" + formatDouble(x_) +
            ",y=" + formatDouble(y_) +
            ",yaw=" + formatDouble(yaw_) +
            ",linear=" + formatDouble(linear_x_) +
            ",angular=" + formatDouble(angular_z_) +
            ",battery=" + std::to_string(battery_) +
            ",status=" + current_status_;

        telemetry_pub_->publish(msg);
    }

    std::string formatDouble(double value)
    {
        char buffer[64];
        std::snprintf(buffer, sizeof(buffer), "%.2f", value);
        return std::string(buffer);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string current_status_;
    double x_;
    double y_;
    double yaw_;
    double linear_x_;
    double angular_z_;
    int battery_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockRobotNode>());
    rclcpp::shutdown();
    return 0;
}
