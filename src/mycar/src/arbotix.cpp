#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ArbotixNode {
public:
    ArbotixNode(): nh_("~") {

        // 设置消息发布频率
        joint_states_pub_rate_ = 10; // 10hz
        odom_pub_rate_ = 10; // 10hz

        // 初始化时间戳
        last_cmd_vel_time_ = ros::Time::now();
        last_joint_states_pub_time_ = ros::Time::now();
        last_odom_pub_time_ = ros::Time::now();

        // 初始化速度和位移
        vx_ = 0;
        vy_ = 0;
        vth_ = 0;
        x_ = 0;
        y_ = 0;
        yaw_ = 0;

        // 订阅/cmd_vel话题
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &ArbotixNode::cmdVelCallback, this);
        // 发布/joint_states话题
        joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
        // 发布/odom话题
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 60);
    }

    void spin() {
        ros::Rate loop_rate(100); // 100hz
        while (ros::ok()) {
            // 获取当前时间
            ros::Time current_time = ros::Time::now();
            
            // 发布/joint_states话题
            if ((current_time - last_joint_states_pub_time_).toSec() > 1.0 / joint_states_pub_rate_) {
                sensor_msgs::JointState joint_state;
                joint_state.header.stamp = current_time;
                joint_state.name.resize(4);
                joint_state.position.resize(4);
                joint_state.velocity.resize(4);
                joint_state.name[0] = "base_l_wheel_joint";
                joint_state.position[0] = left_wheel_pos_;
                joint_state.velocity[0] = left_wheel_vel_;
                joint_state.name[1] = "base_r_wheel_joint";
                joint_state.position[1] = right_wheel_pos_;
                joint_state.velocity[1] = right_wheel_vel_;
                joint_state.name[2] = "front_wheel2base_link";
                joint_state.position[2] = front_wheel_pos_;
                joint_state.velocity[2] = front_wheel_vel_;
                joint_state.name[3] = "back_wheel2base_link";
                joint_state.position[3] = back_wheel_pos_;
                joint_state.velocity[3] = back_wheel_vel_;
                joint_states_pub_.publish(joint_state);

                last_joint_states_pub_time_ = current_time;
            }

            current_time = ros::Time::now();
            // 发布/odom话题
            if ((current_time - last_odom_pub_time_).toSec() > 1.0 / odom_pub_rate_) {
                double dt = (current_time - last_cmd_vel_time_).toSec();
                ROS_INFO("dt = %f", dt);
                double d = (left_wheel_vel_ + right_wheel_vel_) / 2 * dt;
                ROS_INFO("d = %f ", d);
                double delta_th = (right_wheel_vel_ - left_wheel_vel_) / 0.5 * dt;
                x_ += d * cos(yaw_);
                y_ += d * sin(yaw_);
                ROS_INFO("x_ = %f y_ = %f \n", x_, y_);

                yaw_ += delta_th;
                vx_ = (left_wheel_vel_ + right_wheel_vel_) / 2;
                vy_ = 0;
                vth_ = (right_wheel_vel_ - left_wheel_vel_) / 0.5;
                
                geometry_msgs::Quaternion odom_quat = tf2::toMsg(tf2::Quaternion(0, 0, yaw_));
                nav_msgs::Odometry odom;
                odom.header.stamp = current_time;
                odom.header.frame_id = "odom";
                odom.child_frame_id = "base_footprint";
                odom.pose.pose.position.x = x_;
                odom.pose.pose.position.y = y_;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;
                odom.twist.twist.linear.x = vx_;
                odom.twist.twist.linear.y = vy_;
                odom.twist.twist.angular.z = vth_;
                odom_pub_.publish(odom);

                // 发布/tf话题
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_footprint";
                odom_trans.transform.translation.x = x_;
                odom_trans.transform.translation.y = y_;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom_quat;
                tf_broadcaster_.sendTransform(odom_trans);

                last_odom_pub_time_ = current_time;
            }

            // 根据运动学模型计算小车的运动状态
            double dt = (current_time - last_cmd_vel_time_).toSec();
            if (dt > cmd_vel_timeout_) {
                left_wheel_vel_ = 0;
                right_wheel_vel_ = 0;
            }
            // double v = (left_wheel_vel_ + right_wheel_vel_) / 2;
            // double w = (right_wheel_vel_ - left_wheel_vel_) / 0.5;
            // double d = v * dt;
            // double delta_th = w * dt;
            // x_ += d * cos(yaw_);
            // y_ += d * sin(yaw_);
            // yaw_ += delta_th;

            // 更新最近一次/cmd_vel时间戳
            last_cmd_vel_time_ = current_time;

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 获取线速度和角速度
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

        // 根据运动学模型计算小车的运动状态
        double v = linear_x;
        double w = angular_z;
        double r = 0.1;
        double L = 0.5;
        double vl = (v - w * L / 2) / r;
        double vr = (v + w * L / 2) / r;
        left_wheel_vel_ = vl;
        right_wheel_vel_ = vr;

        // 根据关节状态计算小车的位姿
        // double dt = 0.1;
        // double d = (vl + vr) / 2 * dt;
        // double delta_th = (vr - vl) / L * dt;
        // x_ += d * cos(yaw_);
        // y_ += d * sin(yaw_);
        // yaw_ += delta_th;
        // vx_ = v;
        // vy_ = 0;
        // vth_ = w;
    }
    
private:

    double joint_states_pub_rate_;
    double odom_pub_rate_;
    double cmd_vel_timeout_ = 0.1; // 超时时间，单位秒
    ros::Time last_cmd_vel_time_;
    ros::Time last_joint_states_pub_time_;
    ros::Time last_odom_pub_time_;
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher joint_states_pub_;
    ros::Publisher odom_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    double left_wheel_pos_ = 0;
    double left_wheel_vel_ = 0;
    double right_wheel_pos_ = 0;
    double right_wheel_vel_ = 0;
    double front_wheel_pos_ = 0;
    double front_wheel_vel_ = 0;
    double back_wheel_pos_ = 0;
    double back_wheel_vel_ = 0;
    double x_ = 0;
    double y_ = 0;
    double yaw_ = 0;
    double vx_ = 0;
    double vy_ = 0;
    double vth_ = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arbotix_node");
    ArbotixNode node;
    node.spin();
    return 0;
}
