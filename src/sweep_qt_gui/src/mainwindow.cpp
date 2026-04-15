#include "sweep_qt_gui/mainwindow.h"
#include "sweep_qt_gui/routelistdialog.h"
#include "ui_mainwindow.h"

#include <QComboBox>
#include <QDateTime>
#include <QDebug>
#include <QDialog>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileInfo>
#include <QFont>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QListWidget>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QSettings>
#include <QSpinBox>
#include <QTextEdit>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>

#include <functional>

#include <rosidl_runtime_cpp/traits.hpp>

namespace
{

    template <typename T>
    T *findFirstChild(QObject *root, const QStringList &names)
    {
        if (!root) {
            return nullptr;
        }

        for (const QString &name : names) {
            if (auto *obj = root->findChild<T *>(name, Qt::FindChildrenRecursively)) {
                return obj;
            }
        }
        return nullptr;
    }

    template <typename T>
    QList<T *> findAllChildren(QObject *root, const QStringList &names)
    {
        QList<T *> result;
        if (!root) {
            return result;
        }

        for (const QString &name : names) {
            if (auto *obj = root->findChild<T *>(name, Qt::FindChildrenRecursively)) {
                if (!result.contains(obj)) {
                    result.push_back(obj);
                }
            }
        }
        return result;
    }

    QString nowTimeString()
    {
        return QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    }

    QString extractTextByKeys(const QString &text, const QStringList &keys)
    {
        for (const QString &key : keys) {
            const QString pattern =
                QStringLiteral(R"((?:\"|')?%1(?:\"|')?\s*[:=]\s*(?:\"|')?([^\"'\n\r,}]+))")
                    .arg(QRegularExpression::escape(key));

            QRegularExpression re(pattern, QRegularExpression::CaseInsensitiveOption);
            QRegularExpressionMatch match = re.match(text);
            if (match.hasMatch()) {
                return match.captured(1).trimmed();
            }
        }
        return QString();
    }

    double extractDoubleByKeys(const QString &text, const QStringList &keys, bool *okOut = nullptr)
    {
        for (const QString &key : keys) {
            const QString pattern =
                QStringLiteral(R"((?:\"|')?%1(?:\"|')?\s*[:=]\s*([-+]?\d*\.?\d+))")
                    .arg(QRegularExpression::escape(key));

            QRegularExpression re(pattern, QRegularExpression::CaseInsensitiveOption);
            QRegularExpressionMatch match = re.match(text);
            if (match.hasMatch()) {
                bool ok = false;
                double value = match.captured(1).toDouble(&ok);
                if (ok) {
                    if (okOut) {
                        *okOut = true;
                    }
                    return value;
                }
            }
        }

        if (okOut) {
            *okOut = false;
        }
        return 0.0;
    }

    template <typename MsgT>
    QString msgToYamlText(const MsgT &msg)
    {
        return QString::fromStdString(rosidl_generator_traits::to_yaml(msg));
    }

    QPushButton *createPageButton(const QString &text, const QString &objectName, QWidget *parent)
    {
        auto *btn = new QPushButton(text, parent);
        btn->setObjectName(objectName);
        btn->setMinimumHeight(42);
        return btn;
    }

    QLabel *createValueLabel(const QString &defaultText = QStringLiteral("--"), QWidget *parent = nullptr)
    {
        auto *label = new QLabel(defaultText, parent);
        label->setMinimumWidth(100);
        label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        return label;
    }

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->mapStack->setCurrentWidget(ui->pageMap2D);
    initMapView();
    init3DView();

    connect(ui->btnShow2D, &QPushButton::clicked, this, &MainWindow::switchTo2D);

    connect(ui->btnShow3D, &QPushButton::clicked, this, &MainWindow::switchTo3D);

    connect(ui->btnConnectRos, &QPushButton::clicked,
            this, &MainWindow::onConnectRosClicked);

    connect(ui->btnStartTask, &QPushButton::clicked,
            this, &MainWindow::onStartTaskClicked);

    connect(ui->btnPauseTask, &QPushButton::clicked,
            this, &MainWindow::onPauseTaskClicked);

    connect(ui->btnLocate, &QPushButton::clicked,
            this, &MainWindow::onLocateClicked);

    connect(ui->btnContinueTask, &QPushButton::clicked,
            this, &MainWindow::onContinueTaskClicked);

    connect(ui->btnStopTask, &QPushButton::clicked,
            this, &MainWindow::onStopTaskClicked);

    connect(ui->btnBackCharge, &QPushButton::clicked,
            this, &MainWindow::onBackChargeClicked);

    connect(ui->btnFitView, &QPushButton::clicked,
            this, &MainWindow::onFitViewClicked);

    connect(ui->btnResetView, &QPushButton::clicked,
            this, &MainWindow::onResetViewClicked);

    connect(ui->btnShow2D, &QPushButton::clicked,
            this, &MainWindow::onMap2DClicked);

    connect(ui->btnShow3D, &QPushButton::clicked,
            this, &MainWindow::onMap3DClicked);

    connect(ui->btnTopView, &QPushButton::clicked,
            this, &MainWindow::onTopViewClicked);

    connect(ui->btnFrontView, &QPushButton::clicked,
            this, &MainWindow::onFrontViewClicked);

    connect(ui->btnLeftView, &QPushButton::clicked,
            this, &MainWindow::onLeftViewClicked);

    connect(ui->btnRightView, &QPushButton::clicked,
            this, &MainWindow::onRightViewClicked);

    connect(ui->btnIsoView, &QPushButton::clicked,
            this, &MainWindow::onIsoViewClicked);

    connect(ui->btnRouteList, &QPushButton::clicked,
            this, &MainWindow::onRouteListClicked);

    initRos();
    initUi();
    initTopButtons();
    initPageSwitching();
    initParamsPage();
    initCodePages();

    feedback_.rosConnected = false;
    feedback_.vehicleStatus = "待机";
    feedback_.currentTask = "空闲";
    feedback_.modeText = "地图与路径";
    feedback_.batteryPercent = 100;
    feedback_.linearSpeed = 0.0;
    feedback_.angularSpeed = 0.0;
    feedback_.posX = 0.0;
    feedback_.posY = 0.0;
    feedback_.yaw = 0.0;
    feedback_.charging = false;
    feedback_.paused = false;
    feedback_.currentWaypointIndex = -1;
    feedback_.totalWaypoints = 0;

    applyFeedbackToUi();
    // initBatterySystem();
    initSimulatedFeedback();

    ros_spin_timer_ = new QTimer(this);
    connect(ros_spin_timer_, &QTimer::timeout, this, &MainWindow::onRosSpinOnce);
    ros_spin_timer_->start(50);

    appendLog("INFO", "主窗口初始化完成");
}

MainWindow::~MainWindow()
{
    if (ros_spin_timer_) {
        ros_spin_timer_->stop();
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    delete ui;
}

bool MainWindow::loadMapYaml(const QString &yamlPath)
{
    QFile file(yamlPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Failed to open yaml:" << yamlPath;
        return false;
    }

    QTextStream in(&file);
    QFileInfo yamlInfo(yamlPath);
    QDir yamlDir = yamlInfo.dir();

    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();

        if (line.startsWith("image:")) {
            QString imageFile = line.section(':', 1).trimmed();
            mapImagePath_ = yamlDir.filePath(imageFile);
        } else if (line.startsWith("resolution:")) {
            QString value = line.section(':', 1).trimmed();
            mapResolution_ = value.toDouble();
        } else if (line.startsWith("origin:")) {
            // origin 后面的三行通常是:
            // - -44.8
            // - -75
            // - 0
            QString xLine = in.readLine().trimmed();
            QString yLine = in.readLine().trimmed();
            QString zLine = in.readLine().trimmed();
            Q_UNUSED(zLine);

            xLine.remove('-').prepend('-'); // 先不这么处理，下面重写
        }
    }

    file.close();

    // 重新读一次，更稳一点处理 origin
    if (mapImagePath_.isEmpty()) {
        qDebug() << "image path not found in yaml";
        return false;
    }

    QFile file2(yamlPath);
    if (!file2.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Failed to reopen yaml:" << yamlPath;
        return false;
    }

    QTextStream in2(&file2);
    while (!in2.atEnd()) {
        QString line = in2.readLine().trimmed();
        if (line.startsWith("origin:")) {
            QString xLine = in2.readLine().trimmed();
            QString yLine = in2.readLine().trimmed();
            QString zLine = in2.readLine().trimmed();
            Q_UNUSED(zLine);

            // 去掉前面的 "- "
            xLine = xLine;
            yLine = yLine;

            if (xLine.startsWith("- "))
                xLine = xLine.mid(2).trimmed();
            if (yLine.startsWith("- "))
                yLine = yLine.mid(2).trimmed();

            mapOriginX_ = xLine.toDouble();
            mapOriginY_ = yLine.toDouble();
            break;
        }
    }
    file2.close();

    qDebug() << "Map image path:" << mapImagePath_;
    qDebug() << "Map resolution:" << mapResolution_;
    qDebug() << "Map origin:" << mapOriginX_ << mapOriginY_;

    return true;
}

void MainWindow::initMapView()
{
    mapScene_ = new QGraphicsScene(this);
    ui->graphicsViewMap->setScene(mapScene_);

    QString yamlPath = "/home/user_jared/sweep_ws/src/sweep_qt_gui/maps/map.yaml";

    if (!loadMapYaml(yamlPath)) {
        qDebug() << "loadMapYaml failed";
        return;
    }

    QImage img(mapImagePath_);
    if (img.isNull()) {
        qDebug() << "Failed to load map image:" << mapImagePath_;
        return;
    }

    img = img.convertToFormat(QImage::Format_Grayscale8);

    // 增强显示效果：把三值地图重新映射得更直观
    for (int y = 0; y < img.height(); ++y) {
        uchar *line = img.scanLine(y);
        for (int x = 0; x < img.width(); ++x) {
            uchar v = line[x];

            if (v == 0) {
                line[x] = 0; // 障碍物 -> 黑
            } else if (v == 255) {
                line[x] = 255; // 可通行 -> 白
            } else {
                line[x] = 180; // 未知区域 -> 浅灰
            }
        }
    }

    QPixmap pix = QPixmap::fromImage(img);

    mapScene_->clear();
    mapPixmapItem_ = mapScene_->addPixmap(pix);
    mapScene_->setSceneRect(pix.rect());

    // 让显示更明显
    ui->graphicsViewMap->setBackgroundBrush(QBrush(QColor(30, 30, 30)));
    ui->graphicsViewMap->setRenderHint(QPainter::Antialiasing, true);
    ui->graphicsViewMap->setDragMode(QGraphicsView::ScrollHandDrag);
    ui->graphicsViewMap->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    ui->graphicsViewMap->setResizeAnchor(QGraphicsView::AnchorUnderMouse);
    ui->graphicsViewMap->viewport()->installEventFilter(this);

    // 给地图外框画一个红框，方便确认地图位置
    QPen pen(Qt::red);
    pen.setWidth(3);
    mapScene_->addRect(pix.rect(), pen);

    ui->graphicsViewMap->fitInView(mapPixmapItem_, Qt::KeepAspectRatio);
    mapScaleFactor_ = 1.0;
    mapUserZoomed_ = false;
    ui->graphicsViewMap->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    ui->graphicsViewMap->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    qDebug() << "Pixmap size:" << pix.size();
    qDebug() << "View size:" << ui->graphicsViewMap->size();
    qDebug() << "2D map loaded successfully";
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);

    if (mapPixmapItem_ &&
        ui->mapStack->currentWidget() == ui->pageMap2D &&
        !mapUserZoomed_) {
        ui->graphicsViewMap->fitInView(mapPixmapItem_, Qt::KeepAspectRatio);
    }
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->graphicsViewMap->viewport() && event->type() == QEvent::Wheel) {
        QWheelEvent *wheelEvent = static_cast<QWheelEvent *>(event);

        if (!mapPixmapItem_) {
            return false;
        }

        const double zoomInFactor = 1.15;
        const double zoomOutFactor = 1.0 / zoomInFactor;

        if (wheelEvent->angleDelta().y() > 0) {
            ui->graphicsViewMap->scale(zoomInFactor, zoomInFactor);
            mapScaleFactor_ *= zoomInFactor;
        } else {
            ui->graphicsViewMap->scale(zoomOutFactor, zoomOutFactor);
            mapScaleFactor_ *= zoomOutFactor;
        }

        mapUserZoomed_ = true;

        // 防止缩得太小，回到初始自适应状态
        if (mapScaleFactor_ < 0.8) {
            ui->graphicsViewMap->resetTransform();
            ui->graphicsViewMap->fitInView(mapPixmapItem_, Qt::KeepAspectRatio);
            mapScaleFactor_ = 1.0;
            mapUserZoomed_ = false;
        }

        return true;
    }

    return QMainWindow::eventFilter(watched, event);
}

void MainWindow::switchTo2D()
{
    ui->mapStack->setCurrentWidget(ui->pageMap2D);

    if (mapPixmapItem_) {
        ui->graphicsViewMap->fitInView(mapPixmapItem_, Qt::KeepAspectRatio);
    }
}

void MainWindow::switchTo3D()
{
    ui->mapStack->setCurrentWidget(ui->pageMap3D);

    if (vtkWidget_ && pclViewer_) {
        pclViewer_->spinOnce(1, false);
        vtkWidget_->renderWindow()->Render();
    }
}

void MainWindow::init3DView()
{
    auto *layout = new QVBoxLayout(ui->vtkContainer);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    vtkWidget_ = new QVTKOpenGLNativeWidget(ui->vtkContainer);
    layout->addWidget(vtkWidget_);

    vtkRenderWindow_ = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    vtkWidget_->setRenderWindow(vtkRenderWindow_);

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkRenderWindow_->AddRenderer(renderer);

    // 关键：用 renderer + renderWindow 创建，不让它自己弹独立窗口
    pclViewer_.reset(new pcl::visualization::PCLVisualizer(
        renderer,
        vtkRenderWindow_,
        "embedded_viewer",
        false));

    pclViewer_->setupInteractor(vtkWidget_->interactor(), vtkWidget_->renderWindow());

    pclViewer_->setBackgroundColor(0.08, 0.08, 0.10);
    pclViewer_->addCoordinateSystem(1.0);
    pclViewer_->initCameraParameters();

    vtkRenderTimer_ = new QTimer(this);
    connect(vtkRenderTimer_, &QTimer::timeout, this, [this]() {
        if (vtkWidget_ && pclViewer_) {
            pclViewer_->spinOnce(1, false);
            vtkWidget_->renderWindow()->Render();
        }
    });
    vtkRenderTimer_->start(30);

    loadGlobalPointCloud();

    qDebug() << "3D view initialized successfully";
}

void MainWindow::loadGlobalPointCloud()
{
    if (!pclViewer_) {
        qDebug() << "pclViewer_ is null";
        return;
    }

    QString pcdPath = "/home/user_jared/sweep_ws/src/sweep_qt_gui/maps/global.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdPath.toStdString(), *cloud) == -1) {
        qDebug() << "Failed to load PCD:" << pcdPath;
        return;
    }

    qDebug() << "Loaded PCD:" << pcdPath;
    qDebug() << "Point cloud size:" << cloud->size();

    pclViewer_->removeAllPointClouds();
    pclViewer_->removeAllShapes();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud, 255, 255, 255);
    pclViewer_->addPointCloud<pcl::PointXYZ>(cloud, colorHandler, "global_cloud");

    pclViewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "global_cloud");

    pclViewer_->addCoordinateSystem(1.0);
    pclViewer_->resetCamera();

    if (vtkWidget_) {
        vtkWidget_->renderWindow()->Render();
    }
}

void MainWindow::initRos()
{
    if (!rclcpp::ok()) {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    node_ = rclcpp::Node::make_shared("sweep_qt_gui_node");

    task_cmd_pub_ = node_->create_publisher<std_msgs::msg::String>("/ui/task_cmd", 10);
    params_pub_ = node_->create_publisher<std_msgs::msg::String>("/ui/process_params", 10);
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

#if SWEEP_HAS_AGV_STATUS
    status_sub_ = node_->create_subscription<sweep_interfaces::msg::AgvStatus>(
        "/agv/status", 10,
        std::bind(&MainWindow::onAgvStatusMessage, this, std::placeholders::_1));
#else
    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/agv/status", 10,
        std::bind(&MainWindow::onAgvStatusMessage, this, std::placeholders::_1));
#endif

#if SWEEP_HAS_AGV_TELEMETRY
    telemetry_sub_ = node_->create_subscription<sweep_interfaces::msg::AgvTelemetry>(
        "/agv/telemetry", 10,
        std::bind(&MainWindow::onAgvTelemetryMessage, this, std::placeholders::_1));
#else
    telemetry_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/agv/telemetry", 10,
        std::bind(&MainWindow::onAgvTelemetryMessage, this, std::placeholders::_1));
#endif

    setRos2StatusText("已连接");
}

void MainWindow::initUi()
{
    if (ui->listMenu) {
        ui->listMenu->clear();

        ui->listMenu->addItems(QStringList()
                               << "首页总览"
                               << "参数设置"
                               << "地图与路径"
                               << "手动控制"
                               << "状态监控"
                               << "日志报警");

        QFont menuFont = ui->listMenu->font();
        menuFont.setPointSize(14);
        ui->listMenu->setFont(menuFont);

        ui->listMenu->setMinimumWidth(180);
        ui->listMenu->setStyleSheet(
            "QListWidget {"
            "  background-color: white;"
            "  color: black;"
            "  border: 1px solid #bfbfbf;"
            "}"
            "QListWidget::item {"
            "  padding: 10px 8px;"
            "}"
            "QListWidget::item:selected {"
            "  background-color: #dbeafe;"
            "  color: black;"
            "}");

        for (int i = 0; i < ui->listMenu->count(); ++i) {
            QListWidgetItem *item = ui->listMenu->item(i);
            if (item) {
                item->setSizeHint(QSize(item->sizeHint().width(), 42));
            }
        }

        connect(ui->listMenu, &QListWidget::currentRowChanged, this,
                [this](int row) {
                    if (!ui->stackedWidget) {
                        return;
                    }

                    switch (row) {
                        case 0:
                            if (ui->pageHome)
                                ui->stackedWidget->setCurrentWidget(ui->pageHome);
                            break;
                        case 1:
                            if (ui->pageParams)
                                ui->stackedWidget->setCurrentWidget(ui->pageParams);
                            break;
                        case 2:
                            if (ui->pageMap)
                                ui->stackedWidget->setCurrentWidget(ui->pageMap);
                            break;
                        case 3:
                            if (ui->pageManual)
                                ui->stackedWidget->setCurrentWidget(ui->pageManual);
                            break;
                        case 4:
                            if (ui->pageMonitor)
                                ui->stackedWidget->setCurrentWidget(ui->pageMonitor);
                            break;
                        case 5:
                            if (ui->pageLogs)
                                ui->stackedWidget->setCurrentWidget(ui->pageLogs);
                            break;
                        default:
                            break;
                    }
                });

        // 默认选中“地图与路径”
        ui->listMenu->setCurrentRow(2);
    }

    if (ui->stackedWidget && ui->pageMap) {
        ui->stackedWidget->setCurrentWidget(ui->pageMap);
    }
    // if (ui->labelModeValue) {
    // setRobotState(Idle);
    // }

    if (ui->labelBottomInfo) {
        ui->labelBottomInfo->setText("模式：地图与路径");
    }
    if (ui->labelRosStatus) {
        ui->labelRosStatus->setText("--");
    }

    if (ui->labelRobotStatus) {
        ui->labelRobotStatus->setText("--");
    }

    setBatteryText("100%");

    if (ui->labelTask) {
        ui->labelTask->setText("--");
    }

    if (ui->labelSpeedValue) {
        ui->labelSpeedValue->setText("--");
    }

    if (ui->labelPoseValue) {
        ui->labelPoseValue->setText("--");
    }

    if (ui->labelModeValue) {
        ui->labelModeValue->setText("--");
    }
    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：正常");
    }

    if (ui->labelBottomPose) {
        ui->labelBottomPose->setText("当前位置：0.00,0.00");
    }

    if (ui->labelBottomInfo) {
        ui->labelBottomInfo->setText("模式：地图与路径");
    }
    if (ui->btnStartTask)
        ui->btnStartTask->setEnabled(false);
    if (ui->btnLocate)
        ui->btnLocate->setEnabled(false);
    if (ui->btnPauseTask)
        ui->btnPauseTask->setEnabled(false);
    if (ui->btnContinueTask)
        ui->btnContinueTask->setEnabled(false);
    if (ui->btnStopTask)
        ui->btnStopTask->setEnabled(false);
    if (ui->btnBackCharge)
        ui->btnBackCharge->setEnabled(false);
    if (ui->labelViewMode) {
        ui->labelViewMode->setText("二维地图");
    }

    updateMapViewButtons(true);

    feedback_.rosConnected = false;
    feedback_.vehicleStatus = "待机";
    feedback_.currentTask = "空闲";
    feedback_.modeText = "地图与路径";
    feedback_.batteryPercent = 100;
    feedback_.linearSpeed = 0.0;
    feedback_.angularSpeed = 0.0;
    feedback_.posX = 0.0;
    feedback_.posY = 0.0;
    feedback_.yaw = 0.0;
    feedback_.charging = false;
    feedback_.paused = false;
    feedback_.currentWaypointIndex = -1;
    feedback_.totalWaypoints = 0;

    applyFeedbackToUi();
}

void MainWindow::initTopButtons()
{
    connectButtonByNames({"btnConnectRos", "btnConnectRos2", "btnConnectROS2", "btnConnect"},
                         [this]() { onConnectRosClicked(); });

    connectButtonByNames({"btnStartTask", "btnStart", "btnRunTask"},
                         [this]() { onStartTaskClicked(); });

    connectButtonByNames({"btnPauseTask", "btnPause"},
                         [this]() { onPauseTaskClicked(); });

    connectButtonByNames({"btnStopTask", "btnStop"},
                         [this]() { onStopTaskClicked(); });

    connectButtonByNames({"btnBackCharge", "btnGoHome", "btnBackHome", "btnChargeBack"},
                         [this]() { onGoHomeClicked(); });

    if (ui->btnRouteList) {
        connect(ui->btnRouteList, &QPushButton::clicked,
                this, &MainWindow::onRouteListClicked, Qt::UniqueConnection);
    }

    if (ui->btnMultiGoalNav) {
        connect(ui->btnMultiGoalNav, &QPushButton::clicked,
                this, &MainWindow::onMultiGoalNavClicked, Qt::UniqueConnection);
    }

    qDebug() << "[INIT] initTopButtons finished";
}

void MainWindow::initPageSwitching()
{
    connectButtonByNames({"btnMenuHome", "btnNavHome", "btnPageHome", "btnHome"},
                         [this]() { onShowHomePage(); });

    connectButtonByNames({"btnMenuParams", "btnNavParams", "btnPageParams", "btnParams"},
                         [this]() { onShowParamsPage(); });

    connectButtonByNames({"btnMenuMap", "btnNavMap", "btnPageMap", "btnMap"},
                         [this]() { onShowMapPage(); });

    connectButtonByNames({"btnMenuManual", "btnNavManual", "btnPageManual", "btnManual"},
                         [this]() { onShowManualPage(); });

    connectButtonByNames({"btnMenuMonitor", "btnNavMonitor", "btnPageMonitor", "btnMonitor"},
                         [this]() { onShowMonitorPage(); });

    connectButtonByNames({"btnMenuLogs", "btnNavLogs", "btnPageLogs", "btnLogs"},
                         [this]() { onShowLogsPage(); });
}

void MainWindow::initParamsPage()
{
    configureSpin("speedSpin", 0.0, 5.0, 0.1, 2);
    configureSpin("widthSpin", 0.1, 10.0, 0.1, 2);
    configureSpin("intervalSpin", 0.1, 20.0, 0.1, 2);
    configureSpin("lowBatterySpin", 0.0, 100.0, 1.0, 0);

    if (auto *combo = findFirstChild<QComboBox>(this, {"modeCombo"})) {
        if (combo->count() == 0) {
            combo->addItems({"自动", "手动", "回充"});
        }
    }

    if (auto *btn = findFirstChild<QPushButton>(this, {"btnResetParams"})) {
        connect(btn, &QPushButton::clicked, this, &MainWindow::onBtnResetParamsClicked);
    }

    if (auto *btn = findFirstChild<QPushButton>(this, {"btnSaveParams"})) {
        connect(btn, &QPushButton::clicked, this, &MainWindow::onBtnSaveParamsClicked);
    }

    if (auto *btn = findFirstChild<QPushButton>(this, {"btnApplyParams"})) {
        connect(btn, &QPushButton::clicked, this, &MainWindow::onBtnApplyParamsClicked);
    }

    loadParamsFromSettings();
}

void MainWindow::initCodePages()
{
    ensureHomePage();
    ensureMapPage();
    ensureManualPage();
    ensureMonitorPage();
    ensureLogsPage();
}

void MainWindow::ensureHomePage()
{
    if (!ui->pageHome) {
        return;
    }

    if (!ui->pageHome->layout()) {
        auto *layout = new QVBoxLayout(ui->pageHome);

        auto *title = new QLabel("首页总览", ui->pageHome);
        QFont f = title->font();
        f.setPointSize(16);
        f.setBold(true);
        title->setFont(f);

        auto *desc = new QLabel(
            "当前阶段说明：\n"
            "1. 参数设置页已经正式切换为 Qt Creator 版\n"
            "2. 其余页面暂时仍允许由 C++ 代码补充\n"
            "3. 后续可继续逐页迁移到 mainwindow.ui",
            ui->pageHome);
        desc->setWordWrap(true);

        layout->addWidget(title);
        layout->addWidget(desc);
        layout->addStretch();
    }
}

void MainWindow::ensureMapPage()
{
    if (!ui->pageMap) {
        return;
    }

    // 优先使用 Qt Creator 里已经做好的地图页
    auto *mapFrame = findFirstChild<QFrame>(ui->pageMap, {"mapFrame"});
    auto *mapPlaceholder = findFirstChild<QLabel>(ui->pageMap, {"labelMapPlaceholder"});
    auto *routeList = findFirstChild<QListWidget>(ui->pageMap, {"routeListWidget"});
    auto *mapTitle = findFirstChild<QLabel>(ui->pageMap, {"labelMapTitle"});
    auto *routeTitle = findFirstChild<QLabel>(ui->pageMap, {"labelRouteListTitle"});

    if (mapFrame || mapPlaceholder || routeList || mapTitle || routeTitle) {
        if (mapPlaceholder) {
            mapPlaceholder->setAlignment(Qt::AlignCenter);
            mapPlaceholder->setWordWrap(true);
        }

        if (routeList && routeList->count() == 0) {
            routeList->addItems(QStringList()
                                << "P1 (0, 0)"
                                << "P2 (5, 2)"
                                << "P3 (10, 6)"
                                << "P4 (15, 10)");
        }
        return;
    }

    // 下面是旧版兜底逻辑：只有 pageMap 完全没做时才创建
    if (!ui->pageMap->layout()) {
        auto *layout = new QVBoxLayout(ui->pageMap);

        auto *title = new QLabel("地图与路径页面", ui->pageMap);
        QFont titleFont = title->font();
        titleFont.setPointSize(14);
        titleFont.setBold(true);
        title->setFont(titleFont);

        auto *bodyLayout = new QHBoxLayout;

        auto *mapBox = new QFrame(ui->pageMap);
        mapBox->setFrameShape(QFrame::StyledPanel);
        mapBox->setMinimumHeight(240);

        auto *mapLayout = new QVBoxLayout(mapBox);
        auto *mapText = new QLabel("地图显示区（后续接地图 / RViz / 路径显示）", mapBox);
        mapText->setAlignment(Qt::AlignCenter);
        mapText->setWordWrap(true);
        mapLayout->addWidget(mapText);

        auto *routePanel = new QWidget(ui->pageMap);
        auto *routePanelLayout = new QVBoxLayout(routePanel);

        auto *routeTitleFallback = new QLabel("路径点列表", routePanel);
        auto *routeListFallback = new QListWidget(routePanel);
        routeListFallback->addItems(QStringList()
                                    << "P1 (0, 0)"
                                    << "P2 (5, 2)"
                                    << "P3 (10, 6)"
                                    << "P4 (15, 10)");

        routePanelLayout->addWidget(routeTitleFallback);
        routePanelLayout->addWidget(routeListFallback);

        bodyLayout->addWidget(mapBox, 3);
        bodyLayout->addWidget(routePanel, 1);

        layout->addWidget(title);
        layout->addLayout(bodyLayout);
    }
}

void MainWindow::ensureManualPage()
{
    if (!ui->pageManual) {
        return;
    }

    QPushButton *forwardBtn = findFirstChild<QPushButton>(ui->pageManual,
                                                          {"btnForward", "btnManualForward", "btnMoveForward"});
    QPushButton *backwardBtn = findFirstChild<QPushButton>(ui->pageManual,
                                                           {"btnBackward", "btnManualBackward", "btnMoveBackward"});
    QPushButton *leftBtn = findFirstChild<QPushButton>(ui->pageManual,
                                                       {"btnLeft", "btnManualLeft", "btnTurnLeft"});
    QPushButton *rightBtn = findFirstChild<QPushButton>(ui->pageManual,
                                                        {"btnRight", "btnManualRight", "btnTurnRight"});
    QPushButton *stopBtn = findFirstChild<QPushButton>(ui->pageManual,
                                                       {"btnStopMove", "btnManualStop", "btnStopManual"});

    if (!forwardBtn && !backwardBtn && !leftBtn && !rightBtn && !stopBtn) {
        auto *layout = qobject_cast<QVBoxLayout *>(ui->pageManual->layout());
        if (!layout) {
            layout = new QVBoxLayout(ui->pageManual);
        }

        auto *title = new QLabel("手动控制", ui->pageManual);
        QFont f = title->font();
        f.setPointSize(14);
        f.setBold(true);
        title->setFont(f);
        layout->addWidget(title);

        auto *row1 = new QHBoxLayout;
        auto *row2 = new QHBoxLayout;

        leftBtn = createPageButton("左转", "btnLeft", ui->pageManual);
        forwardBtn = createPageButton("前进", "btnForward", ui->pageManual);
        rightBtn = createPageButton("右转", "btnRight", ui->pageManual);
        backwardBtn = createPageButton("后退", "btnBackward", ui->pageManual);
        stopBtn = createPageButton("停止", "btnStopMove", ui->pageManual);

        row1->addWidget(leftBtn);
        row1->addWidget(forwardBtn);
        row1->addWidget(rightBtn);

        row2->addWidget(backwardBtn);
        row2->addWidget(stopBtn);

        layout->addLayout(row1);
        layout->addLayout(row2);
        layout->addStretch();
    }

    bindManualButton({"btnForward", "btnManualForward", "btnMoveForward"},
                     [this]() { onManualForward(); });

    bindManualButton({"btnBackward", "btnManualBackward", "btnMoveBackward"},
                     [this]() { onManualBackward(); });

    bindManualButton({"btnLeft", "btnManualLeft", "btnTurnLeft"},
                     [this]() { onManualLeft(); });

    bindManualButton({"btnRight", "btnManualRight", "btnTurnRight"},
                     [this]() { onManualRight(); });

    bindManualButton({"btnStopMove", "btnManualStop", "btnStopManual"},
                     [this]() { onManualStop(); });
}

void MainWindow::ensureMonitorPage()
{
    if (!ui->pageMonitor) {
        return;
    }

    // 优先使用 Qt Creator 里已经做好的监控页控件
    monitor_x_value_ = findFirstChild<QLabel>(
        ui->pageMonitor, {"labelXValue"});
    monitor_y_value_ = findFirstChild<QLabel>(
        ui->pageMonitor, {"labelYValue"});
    monitor_yaw_value_ = findFirstChild<QLabel>(
        ui->pageMonitor, {"labelYawValue"});
    monitor_linear_vel_value_ = findFirstChild<QLabel>(
        ui->pageMonitor, {"labelLinearVelValue"});
    monitor_angular_vel_value_ = findFirstChild<QLabel>(
        ui->pageMonitor, {"labelAngularVelValue"});
    monitor_battery_value_ = findFirstChild<QLabel>(
        ui->pageMonitor, {"labelMonitorBatteryValue"});
    monitor_status_value_ = findFirstChild<QLabel>(
        ui->pageMonitor, {"labelMonitorStatusValue"});

    // 如果 Qt Creator 版已经做齐了，就直接使用，不再动态创建
    if (monitor_x_value_ &&
        monitor_y_value_ &&
        monitor_yaw_value_ &&
        monitor_linear_vel_value_ &&
        monitor_angular_vel_value_ &&
        monitor_battery_value_ &&
        monitor_status_value_) {
        return;
    }

    // 否则仍然保留旧的代码兜底，防止页面空白
    auto *layout = qobject_cast<QVBoxLayout *>(ui->pageMonitor->layout());
    if (!layout) {
        layout = new QVBoxLayout(ui->pageMonitor);
    }

    auto *group = new QGroupBox("实时状态监控", ui->pageMonitor);
    auto *form = new QFormLayout(group);

    monitor_x_value_ = createValueLabel("--", group);
    monitor_y_value_ = createValueLabel("--", group);
    monitor_yaw_value_ = createValueLabel("--", group);
    monitor_linear_vel_value_ = createValueLabel("--", group);
    monitor_angular_vel_value_ = createValueLabel("--", group);
    monitor_battery_value_ = createValueLabel("--", group);
    monitor_status_value_ = createValueLabel("--", group);

    form->addRow("X:", monitor_x_value_);
    form->addRow("Y:", monitor_y_value_);
    form->addRow("Yaw:", monitor_yaw_value_);
    form->addRow("线速度:", monitor_linear_vel_value_);
    form->addRow("角速度:", monitor_angular_vel_value_);
    form->addRow("电量:", monitor_battery_value_);
    form->addRow("当前状态:", monitor_status_value_);

    layout->addWidget(group);
    layout->addStretch();
}

void MainWindow::ensureLogsPage()
{
    if (!ui->pageLogs) {
        return;
    }

    if (!page_logs_edit_) {
        page_logs_edit_ = findFirstChild<QPlainTextEdit>(
            ui->pageLogs,
            {"pageLogsEdit", "plainTextEditLogs", "plainTextEditLog", "logEdit", "logsEdit"});

        if (!page_logs_edit_) {
            auto *layout = qobject_cast<QVBoxLayout *>(ui->pageLogs->layout());
            if (!layout) {
                layout = new QVBoxLayout(ui->pageLogs);
            }

            auto *title = new QLabel("日志报警", ui->pageLogs);
            QFont f = title->font();
            f.setPointSize(14);
            f.setBold(true);
            title->setFont(f);

            page_logs_edit_ = new QPlainTextEdit(ui->pageLogs);
            page_logs_edit_->setObjectName("pageLogsEdit");
            page_logs_edit_->setReadOnly(true);

            layout->addWidget(title);
            layout->addWidget(page_logs_edit_);
        }
    }
}

void MainWindow::loadParamsFromSettings()
{
    QSettings settings("sweep", "sweep_qt_gui");

    setSpinValue("speedSpin", settings.value("params/speed", 0.80).toDouble());
    setSpinValue("widthSpin", settings.value("params/width", 1.80).toDouble());
    setSpinValue("intervalSpin", settings.value("params/interval", 0.50).toDouble());
    setSpinValue("lowBatterySpin", settings.value("params/low_battery", 20.0).toDouble());

    const QString mode = settings.value("params/mode", "自动").toString();
    setComboText("modeCombo", mode);

    appendLog("INFO", "已加载本地保存参数");
}

void MainWindow::saveParamsToSettings()
{
    QSettings settings("sweep", "sweep_qt_gui");

    settings.setValue("params/speed", spinValue("speedSpin", 0.80));
    settings.setValue("params/width", spinValue("widthSpin", 1.80));
    settings.setValue("params/interval", spinValue("intervalSpin", 0.50));
    settings.setValue("params/low_battery", spinValue("lowBatterySpin", 20.0));
    settings.setValue("params/mode", comboText("modeCombo"));

    appendLog("INFO", "参数已保存到本地");
}

void MainWindow::publishTaskCommand(const QString &cmd)
{
    if (!task_cmd_pub_) {
        appendLog("ERROR", "task_cmd 发布器未初始化");
        return;
    }

    std_msgs::msg::String msg;
    msg.data = cmd.toStdString();
    task_cmd_pub_->publish(msg);

    appendLog("INFO", QString("已发布任务命令: %1").arg(cmd));
}

void MainWindow::publishParams()
{
    if (!params_pub_) {
        appendLog("ERROR", "process_params 发布器未初始化");
        return;
    }

    std_msgs::msg::String msg;
    msg.data = buildParamsJson().toStdString();
    params_pub_->publish(msg);

    appendLog("INFO", QString("已发布参数: %1").arg(buildParamsJson()));
}

void MainWindow::publishManualCmd(double linear_x, double angular_z)
{
    if (!cmd_vel_pub_) {
        appendLog("ERROR", "cmd_vel 发布器未初始化");
        return;
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    cmd_vel_pub_->publish(msg);

    appendLog("INFO",
              QString("已发布手动控制: linear.x=%1, angular.z=%2")
                  .arg(linear_x, 0, 'f', 2)
                  .arg(angular_z, 0, 'f', 2));
}

QString MainWindow::buildParamsJson() const
{
    QJsonObject obj;
    obj["speed"] = spinValue("speedSpin", 0.80);
    obj["width"] = spinValue("widthSpin", 1.80);
    obj["interval"] = spinValue("intervalSpin", 0.50);
    obj["low_battery"] = spinValue("lowBatterySpin", 20.0);
    obj["mode"] = comboText("modeCombo");

    return QString::fromUtf8(QJsonDocument(obj).toJson(QJsonDocument::Compact));
}

void MainWindow::appendLog(const QString &level, const QString &text)
{
    const QString line = QString("[%1] [%2] %3").arg(nowTimeString(), level, text);

    if (page_logs_edit_) {
        page_logs_edit_->appendPlainText(line);
    }

    auto plainEdits = findAllChildren<QPlainTextEdit>(
        this,
        {"textLog",
         "pageLogsEdit",
         "plainTextEditLogs",
         "plainTextEditLog",
         "logEdit",
         "logsEdit",
         "bottomLogEdit"});

    for (auto *edit : plainEdits) {
        if (edit && edit != page_logs_edit_) {
            edit->appendPlainText(line);
        }
    }

    auto textEdits = findAllChildren<QTextEdit>(
        this,
        {"textEditLog", "textEditLogs", "logBrowser", "bottomTextLog"});

    for (auto *edit : textEdits) {
        if (edit) {
            edit->append(line);
        }
    }
}

void MainWindow::setRos2StatusText(const QString &text)
{
    const auto labels = findAllChildren<QLabel>(
        this,
        {"labelRosStatus",
         "lblRos2Status",
         "lblRos2StatusValue",
         "labelRos2Status",
         "labelRos2StatusValue"});

    for (auto *label : labels) {
        if (label) {
            label->setText(text);
        }
    }
}

void MainWindow::setVehicleStatusText(const QString &text)
{
    const auto labels = findAllChildren<QLabel>(
        this,
        {"labelRobotStatus",
         "labelVehicleStatus",
         "labelVehicleStatusValue",
         "lblVehicleStatus",
         "lblVehicleStatusValue"});

    for (auto *label : labels) {
        if (label) {
            label->setText(text);
        }
    }

    if (monitor_status_value_) {
        monitor_status_value_->setText(text);
    }
}

void MainWindow::updateBatteryDisplay()
{
    // 模拟电量变化
    if (isCharging_) {
        batteryLevel_ += 2; // 回充更快一点
        if (batteryLevel_ >= 100) {
            batteryLevel_ = 100;
            isCharging_ = false;
            setRobotState(Idle);
        }
    } else {
        batteryLevel_ -= 1;
        if (batteryLevel_ <= 20) {
            batteryLevel_ = 20;

            // 触发自动回充
            enterChargingMode();
        }
    }

    setBatteryText(QString("%1%").arg(batteryLevel_));
}

void MainWindow::setCurrentTaskText(const QString &text)
{
    const auto labels = findAllChildren<QLabel>(
        this,
        {"labelTask",
         "labelCurrentTask",
         "labelCurrentTaskValue",
         "lblCurrentTask",
         "lblCurrentTaskValue"});

    for (auto *label : labels) {
        if (label) {
            label->setText(text);
        }
    }
}

double MainWindow::spinValue(const QString &objectName, double fallback) const
{
    if (auto *spin = findFirstChild<QDoubleSpinBox>(const_cast<MainWindow *>(this), {objectName})) {
        return spin->value();
    }

    if (auto *spin = findFirstChild<QSpinBox>(const_cast<MainWindow *>(this), {objectName})) {
        return static_cast<double>(spin->value());
    }

    return fallback;
}

void MainWindow::setSpinValue(const QString &objectName, double value)
{
    if (auto *spin = findFirstChild<QDoubleSpinBox>(this, {objectName})) {
        spin->setValue(value);
        return;
    }

    if (auto *spin = findFirstChild<QSpinBox>(this, {objectName})) {
        spin->setValue(static_cast<int>(value));
    }
}

void MainWindow::configureSpin(const QString &objectName,
                               double minValue,
                               double maxValue,
                               double step,
                               int decimals)
{
    if (auto *spin = findFirstChild<QDoubleSpinBox>(this, {objectName})) {
        spin->setRange(minValue, maxValue);
        spin->setSingleStep(step);
        spin->setDecimals(decimals);
        return;
    }

    if (auto *spin = findFirstChild<QSpinBox>(this, {objectName})) {
        spin->setRange(static_cast<int>(minValue), static_cast<int>(maxValue));
        spin->setSingleStep(static_cast<int>(step <= 0.0 ? 1.0 : step));
    }
}

QString MainWindow::comboText(const QString &objectName) const
{
    if (auto *combo = findFirstChild<QComboBox>(const_cast<MainWindow *>(this), {objectName})) {
        return combo->currentText();
    }
    return QString();
}

void MainWindow::setComboText(const QString &objectName, const QString &text)
{
    if (auto *combo = findFirstChild<QComboBox>(this, {objectName})) {
        int index = combo->findText(text);
        if (index >= 0) {
            combo->setCurrentIndex(index);
        } else {
            combo->addItem(text);
            combo->setCurrentIndex(combo->count() - 1);
        }
    }
}

void MainWindow::connectButtonByNames(const QStringList &names, const std::function<void()> &fn)
{
    if (auto *btn = findFirstChild<QPushButton>(this, names)) {
        connect(btn, &QPushButton::clicked, this, [fn]() { fn(); });
    }
}

void MainWindow::bindManualButton(const QStringList &names, const std::function<void()> &fn)
{
    if (auto *btn = findFirstChild<QPushButton>(ui->pageManual, names)) {
        connect(btn, &QPushButton::clicked, this, [fn]() { fn(); });
    }
}

void MainWindow::onRosSpinOnce()
{
    if (node_ && rclcpp::ok()) {
        rclcpp::spin_some(node_);
    }
}

void MainWindow::onBtnResetParamsClicked()
{
    setSpinValue("speedSpin", 0.80);
    setSpinValue("widthSpin", 1.80);
    setSpinValue("intervalSpin", 0.50);
    setSpinValue("lowBatterySpin", 20.0);
    setComboText("modeCombo", "自动");

    appendLog("INFO", "参数已恢复默认值");
}

void MainWindow::onBtnSaveParamsClicked()
{
    saveParamsToSettings();
}

void MainWindow::onBtnApplyParamsClicked()
{
    publishParams();
}

void MainWindow::onConnectRosClicked()
{
    if (ui->labelRosStatus) {
        ui->labelRosStatus->setText("已连接");
    }

    if (ui->labelRobotStatus) {
        setRobotState(Idle);
    }

    if (ui->labelTask) {
        ui->labelTask->setText("空闲");
    }

    if (ui->labelModeValue) {
        setRobotState(Idle);
    }

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：ROS2已连接");
    }

    if (ui->labelBottomInfo) {
        ui->labelBottomInfo->setText("模式：地图与路径");
    }
    if (ui->btnStartTask)
        ui->btnStartTask->setEnabled(true);
    if (ui->btnLocate)
        ui->btnLocate->setEnabled(true);
    if (ui->btnBackCharge)
        ui->btnBackCharge->setEnabled(true);

    if (ui->btnPauseTask)
        ui->btnPauseTask->setEnabled(false);
    if (ui->btnContinueTask)
        ui->btnContinueTask->setEnabled(false);
    if (ui->btnStopTask)
        ui->btnStopTask->setEnabled(false);
}

void MainWindow::onStartTaskClicked()
{

    if (currentState_ == Charging) {
        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：正在回充，暂时不能开始建图");
        }
        return;
    }

    if (currentState_ == Charging)
        return;

    if (ui->labelRosStatus) {
        ui->labelRosStatus->setText("已连接");
    }

    if (ui->labelRobotStatus) {
        setRobotState(Running);
    }

    if (ui->labelTask) {
        ui->labelTask->setText("建图");
    }

    if (ui->labelModeValue) {
        ui->labelModeValue->setText("建图模式");
    }

    if (ui->labelSpeedValue) {
        ui->labelSpeedValue->setText("0.80 m/s");
    }

    if (ui->labelPoseValue) {
        ui->labelPoseValue->setText("X: 0.00  Y: 0.00  Yaw: 0.00");
    }

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：开始建图");
    }

    if (ui->labelBottomInfo) {
        ui->labelBottomInfo->setText("模式：建图模式");
    }
    if (ui->btnPauseTask)
        ui->btnPauseTask->setEnabled(true);
    if (ui->btnContinueTask)
        ui->btnContinueTask->setEnabled(false);
    if (ui->btnStopTask)
        ui->btnStopTask->setEnabled(true);
}

void MainWindow::onPauseTaskClicked()
{
    if (currentState_ == Charging) {
        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：正在充电，不能暂停任务");
        }
        return;
    }

    if (currentState_ != Running) {
        return;
    }

    setRobotState(Paused);
    feedback_.paused = true;
    feedback_.charging = false;

    feedback_.linearSpeed = 0.00;
    feedback_.angularSpeed = 0.00;

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：任务已暂停");
    }
    if (ui->labelBottomInfo) {
        ui->labelBottomInfo->setText("模式：任务暂停");
    }

    applyFeedbackToUi();

    if (ui->btnPauseTask)
        ui->btnPauseTask->setEnabled(false);
    if (ui->btnContinueTask)
        ui->btnContinueTask->setEnabled(true);
}

void MainWindow::onStopTaskClicked()
{
    setRobotState(Idle);

    feedback_.currentTask = "已取消";
    feedback_.linearSpeed = 0.00;
    feedback_.angularSpeed = 0.00;

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：任务已取消");
    }
    if (ui->labelBottomInfo) {
        ui->labelBottomInfo->setText("模式：地图与路径");
    }

    applyFeedbackToUi();

    if (ui->btnPauseTask)
        ui->btnPauseTask->setEnabled(false);
    if (ui->btnContinueTask)
        ui->btnContinueTask->setEnabled(false);
}

void MainWindow::onGoHomeClicked()
{
    setCurrentTaskText("回充中");
    publishTaskCommand("go_home");
    enterChargingMode();
}

void MainWindow::onShowHomePage()
{
    if (ui->stackedWidget && ui->pageHome) {
        ui->stackedWidget->setCurrentWidget(ui->pageHome);
    }
}

void MainWindow::onShowParamsPage()
{
    if (ui->stackedWidget && ui->pageParams) {
        ui->stackedWidget->setCurrentWidget(ui->pageParams);
    }
}

void MainWindow::onShowMapPage()
{
    if (ui->stackedWidget && ui->pageMap) {
        ui->stackedWidget->setCurrentWidget(ui->pageMap);
    }
}

void MainWindow::onShowManualPage()
{
    if (ui->stackedWidget && ui->pageManual) {
        ui->stackedWidget->setCurrentWidget(ui->pageManual);
    }
}

void MainWindow::onShowMonitorPage()
{
    if (ui->stackedWidget && ui->pageMonitor) {
        ui->stackedWidget->setCurrentWidget(ui->pageMonitor);
    }
}

void MainWindow::onShowLogsPage()
{
    if (ui->stackedWidget && ui->pageLogs) {
        ui->stackedWidget->setCurrentWidget(ui->pageLogs);
    }
}

void MainWindow::onManualForward()
{
    const double speed = spinValue("speedSpin", 0.80);
    publishManualCmd(speed, 0.0);
}

void MainWindow::onManualBackward()
{
    const double speed = spinValue("speedSpin", 0.80);
    publishManualCmd(-speed, 0.0);
}

void MainWindow::onManualLeft()
{
    const double speed = spinValue("speedSpin", 0.80);
    publishManualCmd(0.0, speed);
}

void MainWindow::onManualRight()
{
    const double speed = spinValue("speedSpin", 0.80);
    publishManualCmd(0.0, -speed);
}

void MainWindow::onManualStop()
{
    publishManualCmd(0.0, 0.0);
}

#if SWEEP_HAS_AGV_STATUS
// 真实车端状态反馈入口：
// 预留字段：rosConnected / vehicleStatus / currentTask / modeText / batteryPercent / charging / paused
void MainWindow::onAgvStatusMessage(const sweep_interfaces::msg::AgvStatus::SharedPtr msg)
{
    const QString text = msgToYamlText(*msg);

    const QString status = extractTextByKeys(
        text, {"status", "vehicle_status", "robot_status", "state", "current_status"});
    const QString battery = extractTextByKeys(
        text, {"battery", "battery_percent", "battery_percentage", "power"});
    const QString task = extractTextByKeys(
        text, {"current_task", "task", "task_name"});

    if (!status.isEmpty()) {
        setVehicleStatusText(status);
    }
    if (!battery.isEmpty()) {
        setBatteryText(battery);
    }
    if (!task.isEmpty()) {
        setCurrentTaskText(task);
    }
}
#else
void MainWindow::onAgvStatusMessage(const std_msgs::msg::String::SharedPtr msg)
{
    qDebug() << "[ROS STATUS] received";
    const QString text = QString::fromStdString(msg->data).trimmed();

    if (!text.isEmpty() && !text.contains('=') && !text.contains(':')) {
        setVehicleStatusText(text);
        setCurrentTaskText(text);
        if (monitor_status_value_) {
            monitor_status_value_->setText(text);
        }

        if (ui->labelModeValue) {
            setRobotState(Idle);
        }

        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：正常");
        }
        return;
    }

    const QString status = extractTextByKeys(
        text, {"status", "vehicle_status", "robot_status", "state", "current_status"});
    const QString battery = extractTextByKeys(
        text, {"battery", "battery_percent", "battery_percentage", "power"});
    const QString task = extractTextByKeys(
        text, {"current_task", "task", "task_name"});

    if (!status.isEmpty()) {
        setVehicleStatusText(status);
        setCurrentTaskText(status);
        if (monitor_status_value_) {
            monitor_status_value_->setText(status);
        }
    }
    if (!battery.isEmpty()) {
        setBatteryText(battery.endsWith('%') ? battery : battery + "%");
    }
    if (!task.isEmpty()) {
        setCurrentTaskText(task);
    }

    if (ui->labelModeValue) {
        setRobotState(Idle);
    }

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：正常");
    }

    updateBatteryDisplay();
}
#endif

#if SWEEP_HAS_AGV_TELEMETRY
// 真实车端遥测反馈入口：
// 预留字段：posX / posY / yaw / linearSpeed / angularSpeed
void MainWindow::onAgvTelemetryMessage(const sweep_interfaces::msg::AgvTelemetry::SharedPtr msg)
{
    const QString text = msgToYamlText(*msg);

    bool ok = false;

    const double x = extractDoubleByKeys(text, {"x", "pos_x"}, &ok);
    if (ok && monitor_x_value_) {
        monitor_x_value_->setText(QString::number(x, 'f', 2));
    }

    const double y = extractDoubleByKeys(text, {"y", "pos_y"}, &ok);
    if (ok && monitor_y_value_) {
        monitor_y_value_->setText(QString::number(y, 'f', 2));
    }

    const double yaw = extractDoubleByKeys(text, {"yaw", "theta"}, &ok);
    if (ok && monitor_yaw_value_) {
        monitor_yaw_value_->setText(QString::number(yaw, 'f', 2));
    }

    const double linearVel = extractDoubleByKeys(
        text, {"linear_velocity", "linear_vel", "vx", "speed"}, &ok);
    if (ok && monitor_linear_vel_value_) {
        monitor_linear_vel_value_->setText(QString::number(linearVel, 'f', 2));
    }

    const double angularVel = extractDoubleByKeys(
        text, {"angular_velocity", "angular_vel", "wz", "yaw_rate"}, &ok);
    if (ok && monitor_angular_vel_value_) {
        monitor_angular_vel_value_->setText(QString::number(angularVel, 'f', 2));
    }

    const double battery = extractDoubleByKeys(
        text, {"battery", "battery_percent", "battery_percentage"}, &ok);
    if (ok) {
        setBatteryText(QString::number(battery, 'f', 0) + "%");
    }
}
#else
void MainWindow::onAgvTelemetryMessage(const std_msgs::msg::String::SharedPtr msg)
{
    const QString text = QString::fromStdString(msg->data);

    bool xOk = false;
    bool yOk = false;
    bool yawOk = false;
    bool linearOk = false;
    bool angularOk = false;
    bool batteryOk = false;

    const double x = extractDoubleByKeys(text, {"x", "pos_x"}, &xOk);
    const double y = extractDoubleByKeys(text, {"y", "pos_y"}, &yOk);
    const double yaw = extractDoubleByKeys(text, {"yaw", "theta"}, &yawOk);

    const double linearVel = extractDoubleByKeys(
        text, {"linear", "linear_velocity", "linear_vel", "vx", "speed"}, &linearOk);

    const double angularVel = extractDoubleByKeys(
        text, {"angular", "angular_velocity", "angular_vel", "wz", "yaw_rate"}, &angularOk);

    const double battery = extractDoubleByKeys(
        text, {"battery", "battery_percent", "battery_percentage"}, &batteryOk);

    if (xOk && monitor_x_value_) {
        monitor_x_value_->setText(QString::number(x, 'f', 2));
    }
    if (yOk && monitor_y_value_) {
        monitor_y_value_->setText(QString::number(y, 'f', 2));
    }
    if (yawOk && monitor_yaw_value_) {
        monitor_yaw_value_->setText(QString::number(yaw, 'f', 2));
    }

    if (linearOk && monitor_linear_vel_value_) {
        monitor_linear_vel_value_->setText(QString::number(linearVel, 'f', 2));
    }
    if (angularOk && monitor_angular_vel_value_) {
        monitor_angular_vel_value_->setText(QString::number(angularVel, 'f', 2));
    }

    if (batteryOk) {
        const QString batteryText = QString::number(battery, 'f', 0) + "%";
        setBatteryText(batteryText);
        if (monitor_battery_value_) {
            monitor_battery_value_->setText(batteryText);
        }
    }

    const QString status = extractTextByKeys(
        text, {"status", "vehicle_status", "robot_status", "state", "current_status"});
    if (!status.isEmpty()) {
        setVehicleStatusText(status);
        if (monitor_status_value_) {
            monitor_status_value_->setText(status);
        }
    }

    // 右侧系统信息面板新增字段
    if (ui->labelSpeedValue) {
        QString speedText = "--";
        if (linearOk && angularOk) {
            speedText = QString("v:%1  w:%2")
                            .arg(QString::number(linearVel, 'f', 2),
                                 QString::number(angularVel, 'f', 2));
        } else if (linearOk) {
            speedText = QString("v:%1").arg(QString::number(linearVel, 'f', 2));
        }
        ui->labelSpeedValue->setText(speedText);
    }

    if (ui->labelPoseValue) {
        QString poseText = "--";
        if (xOk && yOk && yawOk) {
            poseText = QString("x:%1  y:%2  yaw:%3")
                           .arg(QString::number(x, 'f', 2),
                                QString::number(y, 'f', 2),
                                QString::number(yaw, 'f', 2));
        }
        ui->labelPoseValue->setText(poseText);
    }

    if (ui->labelBottomPose) {
        QString bottomPose = "当前位置：--";
        if (xOk && yOk) {
            bottomPose = QString("当前位置：%1,%2")
                             .arg(QString::number(x, 'f', 2),
                                  QString::number(y, 'f', 2));
        }
        ui->labelBottomPose->setText(bottomPose);
    }
}
#endif

void MainWindow::onLocateClicked()
{
    if (ui->labelRosStatus) {
        ui->labelRosStatus->setText("已连接");
    }

    if (ui->labelRobotStatus) {
        setRobotState(Idle);
    }

    if (ui->labelTask) {
        ui->labelTask->setText("定位");
    }

    if (ui->labelModeValue) {
        ui->labelModeValue->setText("定位模式");
    }

    if (ui->labelSpeedValue) {
        ui->labelSpeedValue->setText("0.00 m/s");
    }

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：进入定位模式");
    }

    if (ui->labelBottomInfo) {
        ui->labelBottomInfo->setText("模式：定位模式");
    }
    if (ui->btnPauseTask)
        ui->btnPauseTask->setEnabled(true);
    if (ui->btnContinueTask)
        ui->btnContinueTask->setEnabled(false);
    if (ui->btnStopTask)
        ui->btnStopTask->setEnabled(true);
}

void MainWindow::onContinueTaskClicked()
{
    if (currentState_ == Charging) {
        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：正在充电，不能继续任务");
        }
        return;
    }

    if (currentState_ != Paused) {
        return;
    }

    setRobotState(Running);
    feedback_.paused = false;
    feedback_.charging = false;

    feedback_.linearSpeed = 0.80;
    feedback_.angularSpeed = 0.10;

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：任务已继续");
    }
    if (ui->labelBottomInfo) {
        ui->labelBottomInfo->setText("模式：导航模式");
    }

    applyFeedbackToUi();

    if (ui->btnPauseTask)
        ui->btnPauseTask->setEnabled(true);
    if (ui->btnContinueTask)
        ui->btnContinueTask->setEnabled(false);
}

void MainWindow::onBackChargeClicked()
{
    if (ui->labelRosStatus) {
        ui->labelRosStatus->setText("已连接");
    }

    if (ui->labelRobotStatus) {
        ui->labelRobotStatus->setText("回充中");
    }

    if (ui->labelTask) {
        ui->labelTask->setText("回充");
    }

    if (ui->labelModeValue) {
        ui->labelModeValue->setText("回充模式");
    }

    if (ui->labelSpeedValue) {
        ui->labelSpeedValue->setText("0.50 m/s");
    }

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：开始回充");
    }

    if (ui->labelBottomInfo) {
        ui->labelBottomInfo->setText("模式：回充模式");
    }
    if (ui->btnPauseTask)
        ui->btnPauseTask->setEnabled(true);
    if (ui->btnContinueTask)
        ui->btnContinueTask->setEnabled(false);
    if (ui->btnStopTask)
        ui->btnStopTask->setEnabled(true);
}

void MainWindow::onFitViewClicked()
{
    if (ui->mapStack && ui->mapStack->currentWidget() == ui->pageMap2D) {
        if (ui->graphicsViewMap && ui->graphicsViewMap->scene()) {
            ui->graphicsViewMap->fitInView(
                ui->graphicsViewMap->scene()->itemsBoundingRect(),
                Qt::KeepAspectRatio);
        }

        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：二维地图已适应视图");
        }
        return;
    }

    if (ui->mapStack && ui->mapStack->currentWidget() == ui->pageMap3D) {
        // 三维下先回到一个较合适的等轴视图
        setPclCameraView(6.0, -6.0, 4.5,
                         0.0, 0.0, 0.0,
                         0.0, 0.0, 1.0);

        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：三维视图已适应显示");
        }
    }
}

void MainWindow::onResetViewClicked()
{
    if (ui->mapStack && ui->mapStack->currentWidget() == ui->pageMap2D) {
        if (ui->graphicsViewMap) {
            ui->graphicsViewMap->resetTransform();

            if (ui->graphicsViewMap->scene()) {
                ui->graphicsViewMap->fitInView(
                    ui->graphicsViewMap->scene()->itemsBoundingRect(),
                    Qt::KeepAspectRatio);
            }
        }

        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：二维地图已重置视角");
        }
        return;
    }

    if (ui->mapStack && ui->mapStack->currentWidget() == ui->pageMap3D) {
        // 三维重置先回到默认等轴视图
        setPclCameraView(6.0, -6.0, 4.5,
                         0.0, 0.0, 0.0,
                         0.0, 0.0, 1.0);

        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：三维视图已重置视角");
        }
    }
}

void MainWindow::onMap2DClicked()
{
    if (ui->mapStack) {
        ui->mapStack->setCurrentWidget(ui->pageMap2D);
    }

    if (ui->labelViewMode) {
        ui->labelViewMode->setText("二维地图");
    }

    updateMapViewButtons(true);

    update3DViewButtons("");

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：切换到二维地图");
    }
}

void MainWindow::onMap3DClicked()
{
    if (ui->mapStack) {
        ui->mapStack->setCurrentWidget(ui->pageMap3D);
    }

    if (ui->labelViewMode) {
        ui->labelViewMode->setText("三维点云");
    }

    updateMapViewButtons(false);

    // 新增：默认给一个稳定的三维视角
    setPclCameraView(6.0, -6.0, 4.5,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 1.0);

    update3DViewButtons("iso");

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：切换到三维视图");
    }
}

void MainWindow::updateMapViewButtons(bool is2D)
{
    if (ui->btnShow2D) {
        if (is2D) {
            ui->btnShow2D->setStyleSheet(
                "font-weight: 600;"
                "background-color: #dbeafe;"
                "border: 1px solid #93c5fd;"
                "padding: 4px 10px;");
        } else {
            ui->btnShow2D->setStyleSheet("");
        }
    }

    if (ui->btnShow3D) {
        if (!is2D) {
            ui->btnShow3D->setStyleSheet(
                "font-weight: 600;"
                "background-color: #dbeafe;"
                "border: 1px solid #93c5fd;"
                "padding: 4px 10px;");
        } else {
            ui->btnShow3D->setStyleSheet("");
        }
    }
}

void MainWindow::setPclCameraView(double px, double py, double pz,
                                  double vx, double vy, double vz,
                                  double ux, double uy, double uz)
{
    if (!pclViewer_) {
        return;
    }

    pclViewer_->setCameraPosition(px, py, pz,
                                  vx, vy, vz,
                                  ux, uy, uz);

    if (ui->vtkContainer) {
        ui->vtkContainer->update();
    }
}

void MainWindow::onTopViewClicked()
{
    if (ui->mapStack && ui->mapStack->currentWidget() != ui->pageMap3D) {
        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：请先切换到三维视图");
        }
        return;
    }

    setPclCameraView(0.0, 0.0, 8.0,
                     0.0, 0.0, 0.0,
                     0.0, 1.0, 0.0);

    update3DViewButtons("top");

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：切换到顶视图");
    }
}

void MainWindow::onFrontViewClicked()
{
    if (ui->mapStack && ui->mapStack->currentWidget() != ui->pageMap3D) {
        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：请先切换到三维视图");
        }
        return;
    }

    setPclCameraView(0.0, -8.0, 2.5,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 1.0);

    update3DViewButtons("front");

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：切换到正视图");
    }
}

void MainWindow::onRightViewClicked()
{
    if (ui->mapStack && ui->mapStack->currentWidget() != ui->pageMap3D) {
        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：请先切换到三维视图");
        }
        return;
    }

    setPclCameraView(8.0, 0.0, 2.5,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 1.0);

    update3DViewButtons("right");

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：切换到右视图");
    }
}

void MainWindow::onIsoViewClicked()
{
    if (ui->mapStack && ui->mapStack->currentWidget() != ui->pageMap3D) {
        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：请先切换到三维视图");
        }
        return;
    }

    setPclCameraView(6.0, -6.0, 4.5,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 1.0);

    update3DViewButtons("iso");

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：切换到等轴视图");
    }
}

void MainWindow::onLeftViewClicked()
{
    if (ui->mapStack && ui->mapStack->currentWidget() != ui->pageMap3D) {
        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：请先切换到三维视图");
        }
        return;
    }

    setPclCameraView(-8.0, 0.0, 2.5,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 1.0);

    update3DViewButtons("left");

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：切换到左视图");
    }
}

void MainWindow::update3DViewButtons(const QString &viewName)
{
    auto resetBtn = [](QPushButton *btn) {
        if (btn)
            btn->setStyleSheet("");
    };

    auto highlightBtn = [](QPushButton *btn) {
        if (btn) {
            btn->setStyleSheet(
                "font-weight: 600;"
                "background-color: #dcfce7;"
                "border: 1px solid #86efac;"
                "padding: 4px 10px;");
        }
    };

    resetBtn(ui->btnTopView);
    resetBtn(ui->btnFrontView);
    resetBtn(ui->btnLeftView);
    resetBtn(ui->btnRightView);
    resetBtn(ui->btnIsoView);

    if (viewName == "top") {
        highlightBtn(ui->btnTopView);
    } else if (viewName == "front") {
        highlightBtn(ui->btnFrontView);
    } else if (viewName == "left") {
        highlightBtn(ui->btnLeftView);
    } else if (viewName == "right") {
        highlightBtn(ui->btnRightView);
    } else if (viewName == "iso") {
        highlightBtn(ui->btnIsoView);
    }
}

void MainWindow::onRouteListClicked()
{
    qDebug() << "[CLICK] onRouteListClicked";

    if (!routeDialog_) {
        routeDialog_ = new RouteListDialog(this);

        connect(routeDialog_, &QObject::destroyed, this, [this]() {
            routeDialog_ = nullptr;
        });

        // ⭐ 只连接一次（非常关键）
        connect(routeDialog_, &RouteListDialog::multiNavStarted, this, [this]() {
            isCharging_ = false;
            setRobotState(Running);

            feedback_.paused = false;
            feedback_.charging = false;
            feedback_.currentTask = "多点导航";
            feedback_.linearSpeed = 0.80;
            feedback_.angularSpeed = 0.10;
            feedback_.currentWaypointIndex = -1;
            feedback_.totalWaypoints = 0;

            applyFeedbackToUi();
        });

        connect(routeDialog_, &RouteListDialog::multiNavProgress, this, [this](const QString &pointName) {
            if (ui->labelBottomStatus) {
                ui->labelBottomStatus->setText(QString("系统状态：正在前往 %1").arg(pointName));
            }
        });

        connect(routeDialog_, &RouteListDialog::multiNavProgressIndex, this,
                [this](int currentIndex, int totalCount, const QString &pointName) {
                    feedback_.currentWaypointIndex = currentIndex;
                    feedback_.totalWaypoints = totalCount;

                    if (ui->labelBottomStatus) {
                        ui->labelBottomStatus->setText(
                            QString("系统状态：正在前往 %1(%2/%3)")
                                .arg(pointName)
                                .arg(currentIndex + 1)
                                .arg(totalCount));
                    }

                    applyFeedbackToUi();
                });

        connect(routeDialog_, &RouteListDialog::multiNavFinished, this, [this]() {
            isCharging_ = false;
            setRobotState(Idle);

            feedback_.paused = false;
            feedback_.charging = false;
            feedback_.currentTask = "空闲";
            feedback_.linearSpeed = 0.00;
            feedback_.angularSpeed = 0.00;

            applyFeedbackToUi();
        });
    }

    routeDialog_->show();
    routeDialog_->raise();
    routeDialog_->activateWindow();
}

void MainWindow::onMultiGoalNavClicked()
{
    qDebug() << "[CLICK] onMultiGoalNavClicked";

    if (currentState_ == Charging) {
        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：正在回充，暂时不能启动多点导航");
        }
        return;
    }

    // ⭐ 先确保窗口存在
    if (!routeDialog_) {
        onRouteListClicked(); // 直接复用
    }

    if (routeDialog_) {
        routeDialog_->show();
        routeDialog_->raise();
        routeDialog_->activateWindow();

        routeDialog_->startMultiNavDirectly(); // ⭐ 关键区别
    }
}

void MainWindow::setRobotState(RobotState state)
{
    qDebug() << "[STATE] setRobotState called, new state =" << state;

    currentState_ = state;

    switch (state) {
        case Idle:
            feedback_.vehicleStatus = "待机";
            feedback_.modeText = "地图与路径";
            feedback_.charging = false;
            feedback_.paused = false;
            break;

        case Running:
            feedback_.vehicleStatus = "作业中";
            feedback_.modeText = "导航模式";
            feedback_.charging = false;
            feedback_.paused = false;
            break;

        case Paused:
            feedback_.vehicleStatus = "暂停中";
            feedback_.modeText = "任务暂停";
            feedback_.charging = false;
            feedback_.paused = true;
            break;

        case Charging:
            feedback_.vehicleStatus = "回充中";
            feedback_.modeText = "充电模式";
            feedback_.charging = true;
            feedback_.paused = false;
            break;
    }

    applyFeedbackToUi();
}

void MainWindow::initBatterySystem()
{
    batteryLevel_ = 100;
    isCharging_ = false;

    updateBatteryDisplay();

    if (!batteryTimer_) {
        batteryTimer_ = new QTimer(this);
    }

    connect(batteryTimer_, &QTimer::timeout, this, [this]() {
        if (currentState_ == Running) {
            batteryLevel_ -= 2;

            if (batteryLevel_ < 0) {
                batteryLevel_ = 0;
            }

            updateBatteryDisplay();

            if (batteryLevel_ <= 20) {
                enterChargingMode();
            }

        } else if (currentState_ == Charging) {
            batteryLevel_ += 5;

            if (batteryLevel_ >= 100) {
                batteryLevel_ = 100;
                updateBatteryDisplay();

                isCharging_ = false;
                setRobotState(Idle);

                if (ui->labelTask) {
                    ui->labelTask->setText("空闲");
                }
                if (ui->labelBottomStatus) {
                    ui->labelBottomStatus->setText("系统状态：充电完成");
                }
                if (ui->labelBottomInfo) {
                    ui->labelBottomInfo->setText("模式：地图与路径");
                }
                return;
            }

            updateBatteryDisplay();
        }
    });

    batteryTimer_->start(1000);
}

void MainWindow::enterChargingMode()
{
    if (currentState_ == Charging) {
        return;
    }

    isCharging_ = true;
    feedback_.charging = true;
    feedback_.paused = false;

    setRobotState(Charging);

    feedback_.currentTask = "回充中";
    feedback_.linearSpeed = 0.00;
    feedback_.angularSpeed = 0.00;

    if (ui->labelBottomStatus) {
        ui->labelBottomStatus->setText("系统状态：自动回充");
    }
    if (ui->labelBottomInfo) {
        ui->labelBottomInfo->setText("模式：充电模式");
    }

    applyFeedbackToUi();
}

void MainWindow::setBatteryText(const QString &text)
{
    if (ui->labelBattery) {
        ui->labelBattery->setText(text);
    }
}

void MainWindow::applyFeedbackToUi()
{
    if (!ui) {
        return;
    }

    if (ui->labelRosStatus) {
        ui->labelRosStatus->setText(feedback_.rosConnected ? "已连接" : "未连接");
    }

    if (ui->labelRobotStatus) {
        ui->labelRobotStatus->setText(feedback_.vehicleStatus);
    }

    if (ui->labelTask) {
        QString taskText = feedback_.currentTask;

        if (feedback_.totalWaypoints > 0 &&
            feedback_.currentWaypointIndex >= 0 &&
            feedback_.currentWaypointIndex < feedback_.totalWaypoints &&
            feedback_.currentTask == "多点导航") {
            taskText += QString("（%1/%2）")
                            .arg(feedback_.currentWaypointIndex + 1)
                            .arg(feedback_.totalWaypoints);
        }

        ui->labelTask->setText(taskText);
    }

    if (ui->labelModeValue) {
        ui->labelModeValue->setText(feedback_.modeText);
    }

    setBatteryText(QString("%1%").arg(feedback_.batteryPercent));

    if (ui->labelSpeedValue) {
        ui->labelSpeedValue->setText(
            QString("%1 m/s").arg(feedback_.linearSpeed, 0, 'f', 2));
    }

    if (ui->labelPoseValue) {
        ui->labelPoseValue->setText(
            QString("X:%1  Y:%2  Yaw:%3")
                .arg(feedback_.posX, 0, 'f', 2)
                .arg(feedback_.posY, 0, 'f', 2)
                .arg(feedback_.yaw, 0, 'f', 2));
    }

    if (ui->labelBottomPose) {
        ui->labelBottomPose->setText(
            QString("当前位置： %1,%2")
                .arg(feedback_.posX, 0, 'f', 2)
                .arg(feedback_.posY, 0, 'f', 2));
    }
}

void MainWindow::initSimulatedFeedback()
{
    if (!useSimulatedFeedback_) {
        return;
    }

    if (!simFeedbackTimer_) {
        simFeedbackTimer_ = new QTimer(this);
    }

    connect(simFeedbackTimer_, &QTimer::timeout, this, [this]() {
        updateSimulatedFeedback();
    });

    simFeedbackTimer_->start(1000);
}

void MainWindow::setFeedbackBattery(int percent)
{
    if (percent < 0)
        percent = 0;
    if (percent > 100)
        percent = 100;

    feedback_.batteryPercent = percent;
    batteryLevel_ = percent; // 兼容现有模拟逻辑
    applyFeedbackToUi();
}

void MainWindow::setFeedbackPose(double x, double y, double yaw)
{
    feedback_.posX = x;
    feedback_.posY = y;
    feedback_.yaw = yaw;
    applyFeedbackToUi();
}

void MainWindow::setFeedbackSpeed(double linear, double angular)
{
    feedback_.linearSpeed = linear;
    feedback_.angularSpeed = angular;
    applyFeedbackToUi();
}

void MainWindow::setFeedbackTask(const QString &task)
{
    feedback_.currentTask = task;
    applyFeedbackToUi();
}

void MainWindow::setFeedbackVehicleStatus(const QString &status)
{
    feedback_.vehicleStatus = status;
    applyFeedbackToUi();
}

void MainWindow::setFeedbackMode(const QString &mode)
{
    feedback_.modeText = mode;
    applyFeedbackToUi();
}

void MainWindow::setFeedbackCharging(bool charging)
{
    feedback_.charging = charging;
    applyFeedbackToUi();
}

void MainWindow::setFeedbackPaused(bool paused)
{
    feedback_.paused = paused;
    applyFeedbackToUi();
}

void MainWindow::updateSimulatedFeedback()
{
    if (!useSimulatedFeedback_) {
        return;
    }

    if (!feedback_.paused && !feedback_.charging && currentState_ == Running) {
        if (feedback_.batteryPercent > 0) {
            feedback_.batteryPercent -= 2;
        }

        feedback_.linearSpeed = 0.80;
        feedback_.angularSpeed = 0.10;

    } else if (feedback_.charging || currentState_ == Charging) {
        if (feedback_.batteryPercent < 100) {
            feedback_.batteryPercent += 5;
        }

        feedback_.linearSpeed = 0.00;
        feedback_.angularSpeed = 0.00;

    } else if (feedback_.paused || currentState_ == Paused) {
        feedback_.linearSpeed = 0.00;
        feedback_.angularSpeed = 0.00;

    } else {
        feedback_.linearSpeed = 0.00;
        feedback_.angularSpeed = 0.00;
    }

    if (feedback_.batteryPercent <= 20 && !feedback_.charging && currentState_ == Running) {
        enterChargingMode();
        return;
    }

    if (feedback_.batteryPercent >= 100 && (feedback_.charging || currentState_ == Charging)) {
        feedback_.batteryPercent = 100;
        feedback_.currentTask = "空闲";
        feedback_.charging = false;
        feedback_.paused = false;

        setRobotState(Idle);

        if (ui->labelBottomStatus) {
            ui->labelBottomStatus->setText("系统状态：充电完成");
        }
        if (ui->labelBottomInfo) {
            ui->labelBottomInfo->setText("模式：地图与路径");
        }

        applyFeedbackToUi();
        return;
    }

    applyFeedbackToUi();
}
