#ifndef ROS2_MANAGER_H
#define ROS2_MANAGER_H

#include <QObject>
#include <QString>
#include <memory>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Ros2Manager : public QObject
{
    Q_OBJECT

public:
    explicit Ros2Manager(QObject *parent = nullptr);
    ~Ros2Manager();

    bool start();
    void stop();
    void sendTaskCommand(const QString &cmd);
    void sendCmdVel(double linear_x, double angular_z);
    void sendProcessParams(const QString &params);

signals:
    void rosConnected(bool ok);
    void statusReceived(const QString &status);
    void telemetryReceived(const QString &telemetry);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr process_params_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;

    std::thread spin_thread_;
    std::atomic<bool> running_;

    void spinLoop();
};

#endif // ROS2_MANAGER_H
