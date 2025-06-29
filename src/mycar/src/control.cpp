#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_Z 0x7a
#define KEYCODE_X 0x78
#define KEYCODE_C 0x63


int kfd = 0;
struct termios cooked, raw;

void init_keyboard()
{
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
}

void restore_keyboard()
{
  tcsetattr(kfd, TCSANOW, &cooked);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandle nh;
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  geometry_msgs::Twist twist;

  int speed = 0, turn = 0;
  bool dirty = false;

  init_keyboard();

  ROS_INFO("Use arrow keys to move the robot.\n");

  while (ros::ok())
  {
    char c;
    speed = 0;
    turn = 0;
    if (read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    switch (c)
    {
    case KEYCODE_W:
      speed++;
      dirty = true;
      break;

    case KEYCODE_S:
      dirty = true;
      break;

    case KEYCODE_A:
      turn++;
      dirty = true;
      break;

    case KEYCODE_D:
      turn--;
      dirty = true;
      break;

    case KEYCODE_Q:
      speed++;
      turn--;
      dirty = true;
      break;

    case KEYCODE_E:
      speed++;
      turn++;
      dirty = true;
      break;

    case KEYCODE_Z:
      speed--;
      turn--;
      dirty = true;
      break;

    case KEYCODE_X:
      speed--;
      dirty = true;
      break;

    case KEYCODE_C:
      speed++;
      turn--;
      dirty = true;
      break;
    }

    twist.linear.x = speed * 0.1;     // 设置线速度
    twist.angular.z = turn * 0.5;     // 设置角速度

    ROS_INFO("speed = %f turn = %f",twist.linear.x, twist.angular.z);

    if (dirty == true)
    {
      twist_pub.publish(twist);
      dirty = false;
    }
  }

  restore_keyboard();

  return 0;
}

