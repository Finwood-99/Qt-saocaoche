#ifndef SWEEP_QT_GUI_ROUTELISTDIALOG_H
#define SWEEP_QT_GUI_ROUTELISTDIALOG_H

#include <QDialog>
#include <QString>
#include <QVector>

class QTableWidget;
class QPushButton;

class RouteListDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RouteListDialog(QWidget *parent = nullptr);
    void startMultiNavDirectly();

signals:
    void multiNavStarted();
    void multiNavProgress(const QString &pointName);
    void multiNavProgressIndex(int currentIndex, int totalCount, const QString &pointName);
    void multiNavFinished();

private slots:
    void onAddPointClicked();
    void onRemovePointClicked();
    void onStartMultiNavClicked();
    void gotoNextPoint();

private:
    struct Waypoint {
        QString name;
        double x;
        double y;
        QString status;
    };

    void initUi();
    void loadMockData();
    void refreshTable();

private:
    QVector<Waypoint> waypoints_;
    int currentIndex_;
    QTableWidget *tableWidget_;

private:
    QPushButton *btnAddPoint_;
    QPushButton *btnStartMultiNav_;
    QPushButton *btnRemovePoint_;
    QPushButton *btnClose_;
};

#endif // SWEEP_QT_GUI_ROUTELISTDIALOG_H
