#include "sweep_qt_gui/ros2_manager.h"
#include <chrono>

using namespace std::chrono_literals;

Ros2Manager::Ros2Manager(QObject *parent)
    : QObject(parent), running_(false)
{
}

Ros2Manager::~Ros2Manager()
{
    stop();
}

bool Ros2Manager::start()
{
    if (running_) {
        emit rosConnected(true);
        return true;
    }

    if (!rclcpp::ok()) {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    node_ = rclcpp::Node::make_shared("sweep_qt_gui_node");

    task_cmd_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/ui/task_cmd", 10);

    process_params_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/ui/process_params", 10);

    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/agv/status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            emit statusReceived(QString::fromStdString(msg->data));
        });

    telemetry_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/agv/telemetry", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            emit telemetryReceived(QString::fromStdString(msg->data));
        });

    running_ = true;
    spin_thread_ = std::thread(&Ros2Manager::spinLoop, this);

    emit rosConnected(true);
    return true;
}

void Ros2Manager::stop()
{
    if (!running_) {
        return;
    }

    running_ = false;

    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }

    status_sub_.reset();
    telemetry_sub_.reset();
    task_cmd_pub_.reset();
    process_params_pub_.reset();
    cmd_vel_pub_.reset();
    node_.reset();

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void Ros2Manager::sendTaskCommand(const QString &cmd)
{
    if (!task_cmd_pub_) {
        emit rosConnected(false);
        return;
    }

    std_msgs::msg::String msg;
    msg.data = cmd.toStdString();
    task_cmd_pub_->publish(msg);
}

void Ros2Manager::sendProcessParams(const QString &params)
{
    if (!process_params_pub_) {
        emit rosConnected(false);
        return;
    }

    std_msgs::msg::String msg;
    msg.data = params.toStdString();
    process_params_pub_->publish(msg);
}

void Ros2Manager::sendCmdVel(double linear_x, double angular_z)
{
    if (!cmd_vel_pub_) {
        emit rosConnected(false);
        return;
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    cmd_vel_pub_->publish(msg);
}

void Ros2Manager::spinLoop()
{
    while (running_ && rclcpp::ok() && node_) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(50ms);
    }
}
