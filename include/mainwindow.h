#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSettings>
#include <QMessageBox>
#include <QStringListModel>
#include <iostream>

#include <ros/ros.h>

// 前向声明
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_button_connect_clicked();
    void on_checkbox_use_environment_stateChanged(int state);
    void on_checkbox_remember_settings_stateChanged(int state);

    void updateLoggingView(const QString &message);

    void on_action_Quit_triggered();
    void on_action_Preferences_triggered();
    void on_actionAbout_triggered();
    void on_actionAbout_Qt_triggered();

    void closeEvent(QCloseEvent *event); // 重写closeEvent

private:
    Ui::MainWindow *ui;
    QSettings settings;
    ros::NodeHandle* rosNode;

    void readSettings();
    void writeSettings();
    void updateEnabledState();
};

#endif // MAINWINDOW_H
