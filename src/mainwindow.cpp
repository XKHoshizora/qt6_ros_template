#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    settings("qt6_ros_template", "MainWindow"),
    rosNode(nullptr)
{
    ui->setupUi(this);
    readSettings();
    updateEnabledState();
}

MainWindow::~MainWindow()
{
    delete ui;
    if (rosNode) {
        delete rosNode;
    }
}

void MainWindow::on_button_connect_clicked()
{
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "qt_ros_node");
    }
    
    if (!rosNode) {
        rosNode = new ros::NodeHandle();
    }

    if (rosNode->ok()) {
        updateLoggingView("Connected to ROS master successfully");
    } else {
        updateLoggingView("Failed to connect to ROS master");
    }
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state)
{
    bool enabled = (state == Qt::Checked);
    ui->line_edit_master->setEnabled(!enabled);
    ui->line_edit_host->setEnabled(!enabled);
}

void MainWindow::on_checkbox_remember_settings_stateChanged(int state)
{
    bool remember = (state == Qt::Checked);
    ui->line_edit_master->setEnabled(!remember);
    ui->line_edit_host->setEnabled(!remember);
    ui->checkbox_use_environment->setEnabled(!remember);
}

void MainWindow::updateLoggingView(const QString &message)
{
    QStringList items;
    items << message;
    ui->view_logging->setModel(new QStringListModel(items));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    writeSettings();
    QMainWindow::closeEvent(event);
}

void MainWindow::on_action_Quit_triggered()
{
    close();
}

void MainWindow::on_action_Preferences_triggered()
{
    QMessageBox::information(this, tr("Preferences"), tr("Preferences dialog not implemented yet."));
}

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("About"), tr("qt6_ros_template - a Qt6 and ROS integration example."));
}

void MainWindow::on_actionAbout_Qt_triggered()
{
    QApplication::aboutQt();
}

void MainWindow::readSettings()
{
    settings.beginGroup("MainWindow");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    ui->checkbox_remember_settings->setChecked(settings.value("remember_settings", false).toBool());
    ui->checkbox_use_environment->setChecked(settings.value("use_environment_variables", true).toBool());
    ui->line_edit_master->setText(settings.value("master_url", "http://192.168.1.2:11311/").toString());
    ui->line_edit_host->setText(settings.value("host_ip", "192.168.1.67").toString());
    settings.endGroup();
}

void MainWindow::writeSettings()
{
    settings.beginGroup("MainWindow");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings", ui->checkbox_remember_settings->isChecked());
    if (ui->checkbox_remember_settings->isChecked())
    {
        settings.setValue("use_environment_variables", ui->checkbox_use_environment->isChecked());
        settings.setValue("master_url", ui->line_edit_master->text());
        settings.setValue("host_ip", ui->line_edit_host->text());
    }
    settings.endGroup();
}

void MainWindow::updateEnabledState()
{
    bool connected = false; // You would typically check your ROS connection status here
    ui->button_connect->setEnabled(!connected);
    ui->line_edit_master->setEnabled(!connected && !ui->checkbox_use_environment->isChecked());
    ui->line_edit_host->setEnabled(!connected && !ui->checkbox_use_environment->isChecked());
    ui->line_edit_topic->setEnabled(!connected);
    ui->checkbox_use_environment->setEnabled(!connected);
}
