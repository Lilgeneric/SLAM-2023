#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "Serial.h"
#include "Serial.cpp"

using namespace std;
Test_receive ser_recv;

// 回调函数
void callback(const geometry_msgs::TwistConstPtr& msg, Serial& serial) 
{
  // 接收消息中的线速度 x 和 y，以及角速度 z

  test_send.flag = 0xAA;
  test_send.vel_x = msg->linear.x;
  test_send.angle_z = msg->angular.z;
/*
  ROS_INFO("%s", to_string(test_send.flag).c_str());
  ROS_INFO("%s", to_string(test_send.angle_z).c_str());
  ROS_INFO("%s", to_string(test_send.vel_x).c_str());
*/
  ROS_INFO("%d", sizeof(test_send.flag));
  ROS_INFO("%d", sizeof(test_send.vel_x));
  ROS_INFO("%d", sizeof(test_send.angle_z));
  ROS_INFO("%d", sizeof(test_send));
  
  // 将消息打包成一串字符串，以便通过串口发送
  // std::string send_data = to_string(vel_x) + to_string(flag) + to_string(vel_z) +  "\n";

  // ROS_INFO("%s", send_data.c_str());

  // 通过串口发送消
  // ser.write(send_data.c_str());

  ser_recv = serial.receive();
  if(ser_recv.mode != 0)
    serial.send(test_send);
}

int main(int argc, char** argv) 
{
    // 初始化 ROS 节点
  ros::init(argc, argv, "serial_publisher");
  ros::NodeHandle nh;
Serial serial;


  // 创建串口对象
  // serial::Serial ser("/dev/ttyS0", 115200, serial::Timeout::simpleTimeout(1000));
  serial.openPort("/dev/ttyTHS2");

  // 创建订阅者对象，订阅 /cmd_vel 话题
  ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(callback, _1, boost::ref(serial)));

  // 循环等待消息
  ros::spin();
}
