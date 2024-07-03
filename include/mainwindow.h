#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// 包含必要的Qt头文件
// Include necessary Qt headers
#include <QDateTime>
#include <QMainWindow>
#include <QMessageBox>
#include <QSettings>
#include <QStringListModel>
#include <QTimer>
#include <iostream>

// 包含必要的ROS头文件
// Include necessary ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unistd.h> // 用于gethostname() / For gethostname()

// 前向声明Ui命名空间中的MainWindow类
// Forward declaration of MainWindow class in Ui namespace
namespace Ui {
class MainWindow;
}

// MainWindow类定义，继承自QMainWindow
// MainWindow class definition, inheriting from QMainWindow
class MainWindow : public QMainWindow {
    Q_OBJECT // 启用Qt的元对象特性 / Enable Qt's meta-object features

        public :
        // 构造函数和析构函数
        // Constructor and destructor
        explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

  private slots:
    // 用户界面相关的槽函数
    // Slots for user interface interactions
    void on_button_connect_clicked();
    void on_checkbox_use_environment_stateChanged(int state);
    void on_checkbox_remember_settings_stateChanged(int state);
    void on_button_publish_clicked();
    void on_button_change_topic_clicked();

    // 更新日志视图和ROS状态的槽函数
    // Slots for updating logging view and ROS status
    void updateLoggingView(const QString &message);
    void updateROSStatus();

    // 菜单操作相关的槽函数
    // Slots for menu actions
    void on_action_Quit_triggered();
    void on_action_Preferences_triggered();
    void on_actionAbout_triggered();
    void on_actionAbout_Qt_triggered();

    // ROS循环处理函数
    // Function for ROS spin once
    void spinOnce();

  private:
    Ui::MainWindow *ui;         // UI对象指针 / Pointer to UI object
    QSettings settings;         // 应用程序设置 / Application settings
    ros::NodeHandle *rosNode;   // ROS节点句柄 / ROS node handle
    ros::Subscriber chatterSub; // ROS订阅者 / ROS subscriber
    ros::Publisher chatterPub;  // ROS发布者 / ROS publisher
    QTimer *rosSpinner;         // ROS自旋定时器 / Timer for ROS spinning
    QTimer *rosStatusTimer; // ROS状态更新定时器 / Timer for updating ROS status
    QString currentTopic; // 当前话题 / Current topic

    // 私有成员函数
    // Private member functions
    void readSettings();       // 读取设置 / Read settings
    void writeSettings();      // 写入设置 / Write settings
    void updateEnabledState(); // 更新启用状态 / Update enabled state
    void initRos();            // 初始化ROS / Initialize ROS
    void chatterCallback(const std_msgs::String::ConstPtr
                             &msg); // ROS回调函数 / ROS callback function
    void connectToROS();            // 连接到ROS / Connect to ROS
    void disconnectFromROS();       // 断开ROS连接 / Disconnect from ROS
    void
    subscribeToTopic(const QString &topic); // 订阅话题 / Subscribe to topic
    void unsubscribeFromTopic(); // 取消话题订阅 / Unsubscribe from topic

    // 重写closeEvent函数，处理窗口关闭事件
    // Override closeEvent function to handle window close event
    void closeEvent(QCloseEvent *event) override;
};

#endif // MAINWINDOW_H