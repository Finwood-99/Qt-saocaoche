#include "sweep_qt_gui/routelistdialog.h"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>

RouteListDialog::RouteListDialog(QWidget *parent)
    : QDialog(parent),
      currentIndex_(0),
      tableWidget_(nullptr),
      btnAddPoint_(nullptr),
      btnStartMultiNav_(nullptr),
      btnRemovePoint_(nullptr),
      btnClose_(nullptr)
{
    initUi();
    loadMockData();
}

void RouteListDialog::initUi()
{
    setWindowTitle("路径点列表");
    resize(760, 420);
    setModal(false);

    auto *mainLayout = new QVBoxLayout(this);
    auto *buttonLayout = new QHBoxLayout();

    tableWidget_ = new QTableWidget(this);
    tableWidget_->setColumnCount(5);
    tableWidget_->setHorizontalHeaderLabels(
        {"编号", "点位名称", "X", "Y", "状态"});
    tableWidget_->horizontalHeader()->setStretchLastSection(true);
    tableWidget_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    tableWidget_->verticalHeader()->setVisible(false);
    tableWidget_->setSelectionBehavior(QAbstractItemView::SelectRows);
    tableWidget_->setSelectionMode(QAbstractItemView::SingleSelection);
    tableWidget_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    tableWidget_->setAlternatingRowColors(true);

    btnAddPoint_ = new QPushButton("添加点位", this);
    btnRemovePoint_ = new QPushButton("删除点位", this);
    btnStartMultiNav_ = new QPushButton("开始多点导航", this);
    btnClose_ = new QPushButton("关闭", this);

    buttonLayout->addWidget(btnAddPoint_);
    buttonLayout->addWidget(btnRemovePoint_);
    buttonLayout->addWidget(btnStartMultiNav_);
    buttonLayout->addStretch();
    buttonLayout->addWidget(btnClose_);

    mainLayout->addWidget(tableWidget_);
    mainLayout->addLayout(buttonLayout);

    connect(btnAddPoint_, &QPushButton::clicked,
            this, &RouteListDialog::onAddPointClicked);
    connect(btnRemovePoint_, &QPushButton::clicked,
            this, &RouteListDialog::onRemovePointClicked);
    connect(btnClose_, &QPushButton::clicked,
            this, &QDialog::close);
    connect(btnStartMultiNav_, &QPushButton::clicked,
            this, &RouteListDialog::onStartMultiNavClicked);
}

void RouteListDialog::loadMockData()
{
    waypoints_.clear();

    waypoints_.push_back({"起点", 0.00, 0.00, "待执行"});
    waypoints_.push_back({"作业点A", 1.25, 2.30, "待执行"});
    waypoints_.push_back({"作业点B", 3.10, 4.80, "待执行"});
    waypoints_.push_back({"终点", 5.00, 6.20, "待执行"});

    refreshTable();
}

void RouteListDialog::onAddPointClicked()
{
    const int index = waypoints_.size() + 1;
    waypoints_.push_back({QString("新点位%1").arg(index), 0.00, 0.00, "待执行"});
    refreshTable();
}

void RouteListDialog::onRemovePointClicked()
{
    const int row = tableWidget_->currentRow();
    if (row >= 0 && row < waypoints_.size()) {
        waypoints_.removeAt(row);
        refreshTable();
    }
}

void RouteListDialog::refreshTable()
{
    tableWidget_->setRowCount(waypoints_.size());

    for (int i = 0; i < waypoints_.size(); ++i) {
        const auto &wp = waypoints_[i];

        tableWidget_->setItem(i, 0, new QTableWidgetItem(QString::number(i + 1)));
        tableWidget_->setItem(i, 1, new QTableWidgetItem(wp.name));
        tableWidget_->setItem(i, 2, new QTableWidgetItem(QString::number(wp.x, 'f', 2)));
        tableWidget_->setItem(i, 3, new QTableWidgetItem(QString::number(wp.y, 'f', 2)));
        tableWidget_->setItem(i, 4, new QTableWidgetItem(wp.status));
    }
}

void RouteListDialog::onStartMultiNavClicked()
{
    if (waypoints_.isEmpty()) {
        return;
    }

    currentIndex_ = 0;

    for (auto &wp : waypoints_) {
        wp.status = "待执行";
    }

    refreshTable();

    emit multiNavStarted();

    gotoNextPoint();
}

void RouteListDialog::gotoNextPoint()
{
    if (currentIndex_ >= waypoints_.size()) {
        setWindowTitle("路径点列表 - 多点导航完成");
        emit multiNavFinished();
        return;
    }

    waypoints_[currentIndex_].status = "执行中";
    refreshTable();
    tableWidget_->selectRow(currentIndex_);

    setWindowTitle(QString("路径点列表 - 正在前往：%1")
                       .arg(waypoints_[currentIndex_].name));

    emit multiNavProgress(waypoints_[currentIndex_].name);
    emit multiNavProgressIndex(currentIndex_, waypoints_.size(),
                               waypoints_[currentIndex_].name);

    QTimer::singleShot(2000, this, [this]() {
        if (currentIndex_ < waypoints_.size()) {
            waypoints_[currentIndex_].status = "已完成";
            refreshTable();

            currentIndex_++;
            gotoNextPoint();
        }
    });
}

void RouteListDialog::startMultiNavDirectly()
{
    onStartMultiNavClicked();
}
