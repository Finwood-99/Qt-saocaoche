#ifndef SWEEP_QT_GUI_MAINWINDOW_H
#define SWEEP_QT_GUI_MAINWINDOW_H

#include <QEvent>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QMainWindow>
#include <QPushButton>
#include <QResizeEvent>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>
#include <QVTKOpenGLNativeWidget.h>
#include <QWheelEvent>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkSmartPointer.h>

#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#if __has_include("sweep_interfaces/msg/agv_status.hpp")
#include "sweep_interfaces/msg/agv_status.hpp"
#define SWEEP_HAS_AGV_STATUS 1
#else
#define SWEEP_HAS_AGV_STATUS 0
#endif

#if __has_include("sweep_interfaces/msg/agv_telemetry.hpp")
#include "sweep_interfaces/msg/agv_telemetry.hpp"
#define SWEEP_HAS_AGV_TELEMETRY 1
#else
#define SWEEP_HAS_AGV_TELEMETRY 0
#endif

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class QLabel;
class QPlainTextEdit;
class RouteListDialog;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    void resizeEvent(QResizeEvent *event) override;
    bool eventFilter(QObject *watched, QEvent *event) override;

private slots:
    void onRosSpinOnce();

    void onBtnResetParamsClicked();
    void onBtnSaveParamsClicked();
    void onBtnApplyParamsClicked();

    void onConnectRosClicked();
    void onStartTaskClicked();
    void onPauseTaskClicked();
    void onLocateClicked();
    void onContinueTaskClicked();
    void onStopTaskClicked();
    void onBackChargeClicked();
    void onGoHomeClicked();

    void onShowHomePage();
    void onShowParamsPage();
    void onShowMapPage();
    void onShowManualPage();
    void onShowMonitorPage();
    void onShowLogsPage();

    void onManualForward();
    void onManualBackward();
    void onManualLeft();
    void onManualRight();
    void onManualStop();
    void onFitViewClicked();
    void onResetViewClicked();
    void onMap2DClicked();
    void onMap3DClicked();
    void updateMapViewButtons(bool is2D);
    void update3DViewButtons(const QString &viewName);

    void onTopViewClicked();
    void onFrontViewClicked();
    void onLeftViewClicked();
    void onRightViewClicked();
    void onIsoViewClicked();

    void onRouteListClicked();

    void onMultiGoalNavClicked();

private:
    enum RobotState {
        Idle,
        Running,
        Paused,
        Charging
    };
    struct AgvFeedback {
        bool rosConnected = false;

        QString vehicleStatus = "--";
        QString currentTask = "--";
        QString modeText = "--";

        int batteryPercent = 100;

        double linearSpeed = 0.0;
        double angularSpeed = 0.0;

        double posX = 0.0;
        double posY = 0.0;
        double yaw = 0.0;

        bool charging = false;
        bool paused = false;

        int currentWaypointIndex = -1;
        int totalWaypoints = 0;
    };

    void setRobotState(RobotState state);
    void initBatterySystem();
    void updateBatteryDisplay();
    void enterChargingMode();

    AgvFeedback feedback_;
    RobotState currentState_ = Idle;
    int batteryLevel_ = 100;
    QTimer *batteryTimer_ = nullptr;
    bool isCharging_ = false;

    bool useSimulatedFeedback_ = true;
    QTimer *simFeedbackTimer_ = nullptr;

private:
    void initMapView();
    bool loadMapYaml(const QString &yamlPath);
    void switchTo2D();
    void switchTo3D();
    void init3DView();
    void loadGlobalPointCloud();
    void setPclCameraView(double px, double py, double pz,
                          double vx, double vy, double vz,
                          double ux, double uy, double uz);

private:
    QGraphicsScene *mapScene_ = nullptr;
    QGraphicsPixmapItem *mapPixmapItem_ = nullptr;

    QString mapImagePath_;
    double mapResolution_ = 0.05;
    double mapOriginX_ = 0.0;
    double mapOriginY_ = 0.0;

    double mapScaleFactor_ = 1.0;
    bool mapUserZoomed_ = false;

    QVTKOpenGLNativeWidget *vtkWidget_ = nullptr;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> vtkRenderWindow_;
    pcl::visualization::PCLVisualizer::Ptr pclViewer_;
    QTimer *vtkRenderTimer_ = nullptr;

private:
    void applyFeedbackToUi();
    void setFeedbackBattery(int percent);
    void setFeedbackPose(double x, double y, double yaw);
    void setFeedbackSpeed(double linear, double angular);
    void setFeedbackTask(const QString &task);
    void setFeedbackVehicleStatus(const QString &status);
    void setFeedbackMode(const QString &mode);
    void setFeedbackCharging(bool charging);
    void setFeedbackPaused(bool paused);

    void initSimulatedFeedback();
    void updateSimulatedFeedback();

    void updateManualControlState(const QString &taskText,
                                  double linearSpeed,
                                  double angularSpeed,
                                  const QString &bottomStatus);

private:
    void initRos();
    void initUi();
    void initTopButtons();
    void initPageSwitching();
    void initParamsPage();
    void initCodePages();

    void ensureHomePage();
    void ensureMapPage();
    void ensureManualPage();
    void ensureMonitorPage();
    void ensureLogsPage();

    void loadParamsFromSettings();
    void saveParamsToSettings();

    void publishTaskCommand(const QString &cmd);
    void publishParams();
    void publishManualCmd(double linear_x, double angular_z);

    QString buildParamsJson() const;
    void appendLog(const QString &level, const QString &text);
    void setRos2StatusText(const QString &text);
    void setVehicleStatusText(const QString &text);
    void setBatteryText(const QString &text);
    void setCurrentTaskText(const QString &text);

    double spinValue(const QString &objectName, double fallback = 0.0) const;
    void setSpinValue(const QString &objectName, double value);
    void configureSpin(const QString &objectName,
                       double minValue,
                       double maxValue,
                       double step,
                       int decimals = 2);
    QString comboText(const QString &objectName) const;
    void setComboText(const QString &objectName, const QString &text);

    void connectButtonByNames(const QStringList &names, const std::function<void()> &fn);
    void bindManualButton(const QStringList &names, const std::function<void()> &fn);

#if SWEEP_HAS_AGV_STATUS
    void onAgvStatusMessage(const sweep_interfaces::msg::AgvStatus::SharedPtr msg);
#else
    void onAgvStatusMessage(const std_msgs::msg::String::SharedPtr msg);
#endif

#if SWEEP_HAS_AGV_TELEMETRY
    void onAgvTelemetryMessage(const sweep_interfaces::msg::AgvTelemetry::SharedPtr msg);
#else
    void onAgvTelemetryMessage(const std_msgs::msg::String::SharedPtr msg);
#endif

private:
    RouteListDialog *routeDialog_ = nullptr;
    Ui::MainWindow *ui;

    QTimer *ros_spin_timer_ = nullptr;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

#if SWEEP_HAS_AGV_STATUS
    rclcpp::Subscription<sweep_interfaces::msg::AgvStatus>::SharedPtr status_sub_;
#else
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
#endif

#if SWEEP_HAS_AGV_TELEMETRY
    rclcpp::Subscription<sweep_interfaces::msg::AgvTelemetry>::SharedPtr telemetry_sub_;
#else
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;
#endif

    QPlainTextEdit *page_logs_edit_ = nullptr;

    QLabel *monitor_x_value_ = nullptr;
    QLabel *monitor_y_value_ = nullptr;
    QLabel *monitor_yaw_value_ = nullptr;
    QLabel *monitor_linear_vel_value_ = nullptr;
    QLabel *monitor_angular_vel_value_ = nullptr;
    QLabel *monitor_battery_value_ = nullptr;
    QLabel *monitor_status_value_ = nullptr;
};

#endif // SWEEP_QT_GUI_MAINWINDOW_H
