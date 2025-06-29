#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "Serial.cpp"

Serial serial;
Test_receive ser_recv;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"pose_publisher");

    //2.创建 ROS 句柄
    ros::NodeHandle nh;

    //3.创建发布者对象
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose",1000);

    //4.组织被发布的消息，编写发布逻辑并发布消息
    geometry_msgs::PoseStamped poseStamped;
    std_msgs::Header header;
    geometry_msgs::Pose pose;
    ros::Rate loop_rate(3);

    while((ser_recv = serial.receive()).flag == 0xAA || ser_recv.mode == 1)
    {
	//前哨站
        if(ser_recv.mode == 1){
            header.frame_id = "";
            header.stamp = ros::Time::now();

            poseStamped.header = header;
            
            pose.position.x = 4.3228931427;
            pose.position.y = -4.47083616257;
            pose.position.z = 0;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0.220006233566;
            pose.orientation.w = 0.975498466012;

            poseStamped.pose = pose;

            // 按照给定的频率发布消息
            ros::Rate loop_rate(10);
	    pose_pub.publish(pose);
	    loop_rate.sleep();
        }
	//启动区
        if(ser_recv.mode == 2){
            header.frame_id = "";
            header.stamp = ros::Time::now();

            poseStamped.header = header;
            
            pose.position.x = 0.0175128411502;
            pose.position.y = -0.018982982263;
            pose.position.z = 0;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = -0.000151599440253;
            pose.orientation.w = 0.999999988509;

            poseStamped.pose = pose;

            // 按照给定的频率发布消息
            ros::Rate loop_rate(10);
	    pose_pub.publish(pose);
	    loop_rate.sleep();
        }

        if(ser_recv.mode == 3){
            header.frame_id = "";
            header.stamp = ros::Time::now();

            poseStamped.header = header;
            
            pose.position.x = 0;
            pose.position.y = 0;
            pose.position.z = 0;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 0;

            poseStamped.pose = pose;

            // 按照给定的频率发布消息
            ros::Rate loop_rate(10);
	    pose_pub.publish(pose);
	    loop_rate.sleep();
        }
    }

    return 0;
}
