#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Header.h>
#include "Serial.h"
#include "Serial.cpp"

Serial serial;
Test_receive ser_recv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_publisher");
    ros::NodeHandle nh;
    serial.openPort("/dev/ttyTHS2");
    
    // std::string port;
    // int baudrate;
    // ros::param::param<std::string>("~port", port, "/dev/ttyUSB0");
    // ros::param::param<int>("~baudrate", baudrate, 115200);
    
    // serial::Serial ser(port, baudrate);
    // if (!ser.isOpen())
    // {
    //     ROS_ERROR("Failed to open serial port");
    //     return -1;
    // }
    
    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    std_msgs::Header header;
    
    while ((ser_recv = serial.receive()).flag == 0xAA || ser_recv.mode == 1)
    {
        // std::string line = ser.readline();
        // std::istringstream ss(line);
        // double x, y, qz;
        // ss >> x >> y >> qz;

        header.frame_id = "odom";
        header.stamp = ros::Time::now();
        transformStamped.header = header;
        transformStamped.child_frame_id = "base_link";

        transformStamped.transform.translation.x = ser_recv.x_d;
        transformStamped.transform.translation.y = ser_recv.y_d;
        transformStamped.transform.translation.z = 0;

        transformStamped.transform.rotation.x = 0;
        transformStamped.transform.rotation.y = 0;
        transformStamped.transform.rotation.z = ser_recv.angle_d;
        transformStamped.transform.rotation.w = 0;

        tf_broadcaster.sendTransform(transformStamped);
        ser_recv.flag = 0;
    }

    return 0;
}
