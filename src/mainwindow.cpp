#include "mainwindow.h"
#include "ui_mainwindow.h"

// 构造函数
// Constructor
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow),
      settings("qt6_ros_template", "MainWindow"), rosNode(nullptr),
      rosSpinner(new QTimer(this)), rosStatusTimer(new QTimer(this)),
      currentTopic("/chatter") {
    std::cout << "MainWindow constructor start" << std::endl;

    ui->setupUi(this);
    std::cout << "UI setup complete" << std::endl;

    readSettings();
    std::cout << "Settings read" << std::endl;

    updateEnabledState();
    std::cout << "Enabled state updated" << std::endl;

    // 设置一个计时器来定期调用 spinOnce，但不立即启动
    // Set up a timer to periodically call spinOnce, but don't start it
    // immediately
    connect(rosSpinner, SIGNAL(timeout()), this, SLOT(spinOnce()));
    std::cout << "ROS spinner prepared" << std::endl;

    // 设置ROS状态更新计时器
    // Set up ROS status update timer
    connect(rosStatusTimer, SIGNAL(timeout()), this, SLOT(updateROSStatus()));
    rosStatusTimer->start(
        1000); // 每秒更新一次ROS状态 / Update ROS status every second

    ui->line_edit_topic->setText(currentTopic);

    std::cout << "MainWindow constructor end" << std::endl;
}

// 析构函数
// Destructor
MainWindow::~MainWindow() {
    std::cout << "MainWindow destructor start" << std::endl;

    delete ui;
    if (rosNode) {
        delete rosNode;
    }

    std::cout << "MainWindow destructor end" << std::endl;
}

// 连接按钮点击事件处理函数
// Connect button click event handler
void MainWindow::on_button_connect_clicked() {
    std::cout << "Connect button clicked" << std::endl;

    if (!ros::isInitialized()) {
        std::cout << "Initializing ROS..." << std::endl;
        initRos();
    }

    if (!rosNode) {
        std::cout << "Connecting to ROS..." << std::endl;
        connectToROS();
    } else {
        std::cout << "Disconnecting from ROS..." << std::endl;
        disconnectFromROS();
    }
}

// 连接到ROS
// Connect to ROS
void MainWindow::connectToROS() {
    std::cout << "Attempting to connect to ROS..." << std::endl;
    updateLoggingView("Attempting to connect to ROS...");

    // 检查 ROS_MASTER_URI 环境变量
    // Check ROS_MASTER_URI environment variable
    char *ros_master_uri = getenv("ROS_MASTER_URI");
    if (ros_master_uri) {
        std::cout << "ROS_MASTER_URI is set to: " << ros_master_uri
                  << std::endl;
        updateLoggingView(
            QString("ROS_MASTER_URI is set to: %1").arg(ros_master_uri));
    } else {
        std::cout << "ROS_MASTER_URI is not set!" << std::endl;
        updateLoggingView("ROS_MASTER_URI is not set!");
        return;
    }

    // 检查 ROS_IP 环境变量
    // Check ROS_IP environment variable
    char *ros_ip = getenv("ROS_IP");
    if (ros_ip) {
        std::cout << "ROS_IP is set to: " << ros_ip << std::endl;
        updateLoggingView(QString("ROS_IP is set to: %1").arg(ros_ip));
    } else {
        std::cout << "ROS_IP is not set. This is not critical, but you might "
                     "want to set it for network communication."
                  << std::endl;
        updateLoggingView("ROS_IP is not set. This is not critical, but you "
                          "might want to set it for network communication.");
    }

    try {
        // 检查ROS主节点
        // Check ROS master
        std::cout << "Checking ROS master..." << std::endl;
        updateLoggingView("Checking ROS master...");
        if (!ros::master::check()) {
            std::cout << "Unable to contact ROS master" << std::endl;
            updateLoggingView(
                "Unable to contact ROS master. Is roscore running?");
            return;
        }
        std::cout << "ROS master is available" << std::endl;
        updateLoggingView("ROS master is available");

        // 创建ROS节点句柄
        // Create ROS node handle
        std::cout << "Creating new ros::NodeHandle" << std::endl;
        updateLoggingView("Creating new ros::NodeHandle");
        rosNode = new ros::NodeHandle();

        // 检查节点是否已注册
        // Check if the node is registered with the master
        std::cout << "Checking if node is registered with master..."
                  << std::endl;
        updateLoggingView("Checking if node is registered with master...");
        if (rosNode->ok()) {
            std::cout << "Node registered with master" << std::endl;
            updateLoggingView("Node registered with master");
        } else {
            std::cout << "Failed to register node with master" << std::endl;
            updateLoggingView("Failed to register node with master");
            delete rosNode;
            rosNode = nullptr;
            return;
        }

        // 订阅话题
        // Subscribe to topic
        std::cout << "Subscribing to topic: " << currentTopic.toStdString()
                  << std::endl;
        updateLoggingView(
            QString("Subscribing to topic: %1").arg(currentTopic));
        subscribeToTopic(currentTopic);

        // 创建发布者
        // Create publisher
        std::cout << "Creating publisher for topic: "
                  << currentTopic.toStdString() << std::endl;
        updateLoggingView(
            QString("Creating publisher for topic: %1").arg(currentTopic));
        chatterPub = rosNode->advertise<std_msgs::String>(
            currentTopic.toStdString(), 1000);

        // 启动ROS自旋定时器
        // Start ROS spinner timer
        rosSpinner->start(500); // 每500毫秒自旋一次 / Spin every 500ms
        std::cout << "ROS spinner started" << std::endl;
        updateLoggingView("ROS spinner started");

        std::cout << "Connected to ROS master successfully" << std::endl;
        updateLoggingView("Connected to ROS master successfully");
        updateEnabledState();
    } catch (ros::Exception &e) {
        std::cout << "Error connecting to ROS master: " << e.what()
                  << std::endl;
        updateLoggingView(
            QString("Error connecting to ROS master: %1").arg(e.what()));
        if (rosNode) {
            delete rosNode;
            rosNode = nullptr;
        }
    }
}

// 从ROS断开连接
// Disconnect from ROS
void MainWindow::disconnectFromROS() {
    if (rosNode) {
        unsubscribeFromTopic();
        chatterPub.shutdown();
        rosSpinner->stop();
        delete rosNode;
        rosNode = nullptr;
        QString message = QString("[%1] Disconnected from ROS master")
                              .arg(QDateTime::currentDateTime().toString(
                                  "yyyy-MM-dd hh:mm:ss"));
        updateLoggingView(message);
        updateEnabledState();
    }
}

// 订阅话题
// Subscribe to topic
void MainWindow::subscribeToTopic(const QString &topic) {
    unsubscribeFromTopic();
    try {
        chatterSub = rosNode->subscribe(topic.toStdString(), 1000,
                                        &MainWindow::chatterCallback, this);
        updateLoggingView(QString("Subscribed to topic: %1").arg(topic));
    } catch (ros::Exception &e) {
        updateLoggingView(QString("Error subscribing to topic %1: %2")
                              .arg(topic)
                              .arg(e.what()));
    }
}

// 取消订阅话题
// Unsubscribe from topic
void MainWindow::unsubscribeFromTopic() {
    if (chatterSub) {
        chatterSub.shutdown();
    }
}

// 发布按钮点击事件处理函数
// Publish button click event handler
void MainWindow::on_button_publish_clicked() {
    if (rosNode && chatterPub) {
        std_msgs::String msg;
        msg.data = ui->line_edit_message->text().toStdString();
        chatterPub.publish(msg);
        updateLoggingView(QString("Published message: %1")
                              .arg(ui->line_edit_message->text()));
    } else {
        updateLoggingView("Cannot publish: Not connected to ROS master");
    }
}

// 更改话题按钮点击事件处理函数
// Change topic button click event handler
void MainWindow::on_button_change_topic_clicked() {
    QString newTopic = ui->line_edit_topic->text();
    if (newTopic != currentTopic) {
        currentTopic = newTopic;
        if (rosNode) {
            subscribeToTopic(currentTopic);
            chatterPub = rosNode->advertise<std_msgs::String>(
                currentTopic.toStdString(), 1000);
        }
    }
}

// 更新ROS状态
// Update ROS status
void MainWindow::updateROSStatus() {
    if (ros::isInitialized() && rosNode && ros::master::check()) {
        ui->label_ros_status->setText("Connected");
        ui->label_ros_status->setStyleSheet("QLabel { color : green; }");
    } else {
        ui->label_ros_status->setText("Disconnected");
        ui->label_ros_status->setStyleSheet("QLabel { color : red; }");
    }
}

// 使用环境变量复选框状态改变事件处理函数
// Use environment checkbox state changed event handler
void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
    bool enabled = (state == Qt::Checked);
    ui->line_edit_master->setEnabled(!enabled);
    ui->line_edit_host->setEnabled(!enabled);

    if (enabled) {
        // 使用环境变量设置 ROS_MASTER_URI
        // Use environment variable to set ROS_MASTER_URI
        QString master_uri = QString::fromStdString(ros::master::getURI());
        if (master_uri.isEmpty()) {
            master_uri = "http://localhost:11311";
        }
        ui->line_edit_master->setText(master_uri);

        // 获取 ROS_IP
        // Get ROS_IP
        char *ros_ip = getenv("ROS_IP");
        if (ros_ip) {
            ui->line_edit_host->setText(QString(ros_ip));
        } else {
            ui->line_edit_host->setText(
                "127.0.0.1"); // 默认本地 IP / Default local IP
        }
    }
}

// 记住设置复选框状态改变事件处理函数
// Remember settings checkbox state changed event handler
void MainWindow::on_checkbox_remember_settings_stateChanged(int state) {
    bool remember = (state == Qt::Checked);
    ui->line_edit_master->setEnabled(!remember);
    ui->line_edit_host->setEnabled(!remember);
    ui->checkbox_use_environment->setEnabled(!remember);
}

// 更新日志视图
// Update logging view
void MainWindow::updateLoggingView(const QString &message) {
    std::cout << message.toStdString()
              << std::endl; // 添加这行来在控制台输出日志 / Add this line to
                            // output log to console
    QStringListModel *model =
        qobject_cast<QStringListModel *>(ui->view_logging->model());
    if (!model) {
        model = new QStringListModel(this);
        ui->view_logging->setModel(model);
    }
    QStringList list = model->stringList();
    list.append(message);
    model->setStringList(list);
    ui->view_logging->scrollToBottom();
}

// 窗口关闭事件处理函数
// Window close event handler
void MainWindow::closeEvent(QCloseEvent *event) {
    std::cout << "closeEvent called" << std::endl;
    writeSettings();
    QMainWindow::closeEvent(event);
}

// 退出动作触发事件处理函数
// Quit action triggered event handler
void MainWindow::on_action_Quit_triggered() { close(); }

// 偏好设置动作触发事件处理函数
// Preferences action triggered event handler
void MainWindow::on_action_Preferences_triggered() {
    QMessageBox::information(this, tr("Preferences"),
                             tr("Preferences dialog not implemented yet."));
}

// 关于动作触发事件处理函数
// About action triggered event handler
void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(
        this, tr("About"),
        tr("qt6_ros_template - a Qt6 and ROS integration example."));
}

// 关于Qt动作触发事件处理函数
// About Qt action triggered event handler
void MainWindow::on_actionAbout_Qt_triggered() { QApplication::aboutQt(); }

// 读取设置
// Read settings
void MainWindow::readSettings() {
    settings.beginGroup("MainWindow");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    ui->checkbox_remember_settings->setChecked(
        settings.value("remember_settings", false).toBool());

    bool use_environment =
        settings.value("use_environment_variables", true).toBool();
    ui->checkbox_use_environment->setChecked(use_environment);

    if (!use_environment) {
        ui->line_edit_master->setText(
            settings.value("master_url", "http://localhost:11311").toString());
        ui->line_edit_host->setText(
            settings.value("host_ip", "localhost").toString());
    } else {
        on_checkbox_use_environment_stateChanged(Qt::Checked);
    }

    settings.endGroup();
}

// 写入设置
// Write settings
void MainWindow::writeSettings() {
    settings.beginGroup("MainWindow");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",
                      ui->checkbox_remember_settings->isChecked());
    settings.setValue("use_environment_variables",
                      ui->checkbox_use_environment->isChecked());
    if (!ui->checkbox_use_environment->isChecked()) {
        settings.setValue("master_url", ui->line_edit_master->text());
        settings.setValue("host_ip", ui->line_edit_host->text());
    }

    settings.endGroup();
}

// 更新UI元素的启用状态
// Update the enabled state of UI elements
void MainWindow::updateEnabledState() {
    bool connected = (rosNode != nullptr);
    ui->button_connect->setText(connected ? "Disconnect" : "Connect");
    ui->button_publish->setEnabled(connected);
    ui->button_change_topic->setEnabled(connected);
    ui->line_edit_message->setEnabled(connected);
    ui->line_edit_topic->setEnabled(connected);
    std::cout << "Updated enabled state. Connected: "
              << (connected ? "true" : "false") << std::endl;
}

// ROS自旋函数,定期被调用以处理ROS消息
// ROS spin function, called periodically to process ROS messages
void MainWindow::spinOnce() {
    if (ros::ok()) {
        if (!ros::master::check()) {
            std::cout << "Lost connection to ROS master" << std::endl;
            QString message = QString("[%1] Lost connection to ROS master")
                                  .arg(QDateTime::currentDateTime().toString(
                                      "yyyy-MM-dd hh:mm:ss"));
            updateLoggingView(message);
            disconnectFromROS();
        } else {
            ros::spinOnce();
        }
    } else {
        std::cout << "ROS is not ok, stopping spinner" << std::endl;
        QString message = QString("[%1] ROS is not ok, stopping spinner")
                              .arg(QDateTime::currentDateTime().toString(
                                  "yyyy-MM-dd hh:mm:ss"));
        updateLoggingView(message);
        rosSpinner->stop();
        disconnectFromROS();
    }
}

// ROS话题回调函数,当接收到消息时被调用
// ROS topic callback function, called when a message is received
void MainWindow::chatterCallback(const std_msgs::String::ConstPtr &msg) {
    std::cout << "Received message in chatterCallback: " << msg->data
              << std::endl;
    QString message = QString("Received: %1").arg(msg->data.c_str());

    // 确保在主线程中更新 UI
    // Ensure UI is updated in the main thread
    updateLoggingView(message);
    std::cout << "Invoked updateLoggingView" << std::endl;
}

// 初始化ROS
// Initialize ROS
void MainWindow::initRos() {
    std::map<std::string, std::string> remappings;
    remappings["__master"] = ui->line_edit_master->text().toStdString();
    remappings["__ip"] = ui->line_edit_host->text().toStdString();

    if (!ros::isInitialized()) {
        ros::init(remappings, "qt_ros_node",
                  ros::init_options::NoSigintHandler);
    }
}

// 写入设置
// Write settings
void MainWindow::writeSettings() {
    settings.beginGroup("MainWindow");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",
                      ui->checkbox_remember_settings->isChecked());
    settings.setValue("use_environment_variables",
                      ui->checkbox_use_environment->isChecked());
    if (!ui->checkbox_use_environment->isChecked()) {
        settings.setValue("master_url", ui->line_edit_master->text());
        settings.setValue("host_ip", ui->line_edit_host->text());
    }
    settings.endGroup();
}

// 更新UI元素的启用状态
// Update the enabled state of UI elements
void MainWindow::updateEnabledState() {
    bool connected = (rosNode != nullptr);
    ui->button_connect->setText(connected ? "Disconnect" : "Connect");
    ui->button_publish->setEnabled(connected);
    ui->button_change_topic->setEnabled(connected);
    ui->line_edit_message->setEnabled(connected);
    ui->line_edit_topic->setEnabled(connected);
    std::cout << "Updated enabled state. Connected: "
              << (connected ? "true" : "false") << std::endl;
}

// ROS自旋函数,定期被调用以处理ROS消息
// ROS spin function, called periodically to process ROS messages
void MainWindow::spinOnce() {
    if (ros::ok()) {
        if (!ros::master::check()) {
            std::cout << "Lost connection to ROS master" << std::endl;
            QString message = QString("[%1] Lost connection to ROS master")
                                  .arg(QDateTime::currentDateTime().toString(
                                      "yyyy-MM-dd hh:mm:ss"));
            updateLoggingView(message);
            disconnectFromROS();
        } else {
            ros::spinOnce();
        }
    } else {
        std::cout << "ROS is not ok, stopping spinner" << std::endl;
        QString message = QString("[%1] ROS is not ok, stopping spinner")
                              .arg(QDateTime::currentDateTime().toString(
                                  "yyyy-MM-dd hh:mm:ss"));
        updateLoggingView(message);
        rosSpinner->stop();
        disconnectFromROS();
    }
}

// ROS话题回调函数,当接收到消息时被调用
// ROS topic callback function, called when a message is received
void MainWindow::chatterCallback(const std_msgs::String::ConstPtr &msg) {
    std::cout << "Received message in chatterCallback: " << msg->data
              << std::endl;
    QString message = QString("Received: %1").arg(msg->data.c_str());

    // 确保在主线程中更新 UI
    // Ensure UI is updated in the main thread
    updateLoggingView(message);
    std::cout << "Invoked updateLoggingView" << std::endl;
}

// 初始化ROS
// Initialize ROS
void MainWindow::initRos() {
    std::map<std::string, std::string> remappings;
    remappings["__master"] = ui->line_edit_master->text().toStdString();
    remappings["__ip"] = ui->line_edit_host->text().toStdString();

    if (!ros::isInitialized()) {
        ros::init(remappings, "qt_ros_node",
                  ros::init_options::NoSigintHandler);
    }
}
