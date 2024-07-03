#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    // 初始化ROS节点
    // Initialize ROS node
    ros::init(argc, argv, "qt6_ros_template_node");

    // 创建节点句柄
    // Create a node handle
    ros::NodeHandle n;

    // 创建一个发布者,发布到"chatter"话题
    // Create a publisher that publishes to the "chatter" topic
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    // 设置循环频率为10Hz
    // Set the loop rate to 10Hz
    ros::Rate loop_rate(10);

    // 主循环
    // Main loop
    while (ros::ok()) {
        // 创建一个String类型的消息
        // Create a String message
        std_msgs::String msg;
        msg.data = "Hello, ROS!";

        // 发布消息
        // Publish the message
        chatter_pub.publish(msg);

        // 处理回调函数
        // Process callbacks
        ros::spinOnce();

        // 睡眠以维持循环频率
        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}