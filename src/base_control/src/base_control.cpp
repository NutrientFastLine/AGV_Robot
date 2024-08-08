#include <ros/ros.h>
#include <serial/serial.h>
#include <string>
#include <stdio.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#define PI 3.1415926
using namespace std;
double angle_pos = 0;
double start_l = 0, start_r = 0, start_angle = 0; // 机器人初始位姿定义,xy坐标，angle角度
serial::Serial sp;                                // 底盘串口定义
struct senddata
{
    unsigned char s[13] = {0}; // 定义一个长度为13的数组
};
uint8_t App_Sum(uint8_t *puCFrame, uint8_t Len) // 校验计算和函数
{
    uint8_t i = 0;
    uint16_t sum = 0;
    for (int i = 0; i < Len; i++)
    {
        sum += puCFrame[i];
    }
    return sum & 0xff;
}
int16_t Two_OCT_turn_HEX(uint8_t DATA1, uint8_t DATA2) // 高8位的速度和低8位的速度信息融合
{
    int16_t DATA_16 = (DATA1 << 8) | DATA2; // DATA1在高位，DATA2在低位
    return DATA_16;
}
void Limit_check(int16_t &mach, double &start, double param)
{
    if (mach * start < 0 && (abs(mach) > 30000))
    {
        if (mach < 0)
        {
            start = (start / param - 2 * 32767) * param;
        }
        else
        {
            start = (start / param + 2 * 32767) * param;
        }
    }
    else
    {
        start = start;
    }
}
void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
    float command_left, command_right;
    command_left = (twist_aux.linear.x - (twist_aux.angular.z * 0.15)); // 0.15为两车轮距离的二分之一；
    command_right = (twist_aux.linear.x + (twist_aux.angular.z * 0.15));
    // cout << cmd_vel << endl;
    struct senddata vel;
    vel.s[0] = 0xAA; // 包头1
    vel.s[1] = 0x55; // 包头2
    vel.s[2] = 0x01; // 地址
    vel.s[3] = 0x03; // 两个轮子线速度独立控制模式
    vel.s[4] = 0x04; // 数据长度
    int16_t num1 = -command_left * 1000;
    vel.s[5] = (uint8_t)((num1 & 0xff00) >> 8); // 轮1高八位
    vel.s[6] = (uint8_t)(num1 & 0x00ff);        // 轮1低八位
    int16_t num2 = -command_right * 1000;
    vel.s[7] = (uint8_t)((num2 & 0xff00) >> 8); // 轮2高八位
    vel.s[8] = (uint8_t)(num2 & 0x00ff);        // 轮2低八位
    vel.s[9] = 0x02;
    vel.s[10] = 0x00;
    vel.s[11] = App_Sum(vel.s, 11); // 校验位计算
    vel.s[12] = 0xCC;
    Two_OCT_turn_HEX(vel.s[5], vel.s[6]);
    sp.write(vel.s, sizeof(vel.s));
}
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "base_control");
    ros::NodeHandle n;
    double diameter, param;
    ros::param::get("~param", param);
    ros::param::get("~diameter", diameter);
    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 10, cmd_velCallback); // 订阅上位机速度
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);     // 发布机器人里程计信息
    try
    {
        sp.setPort("/dev/tiewoniu_base");
        sp.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        sp.setTimeout(to);
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open tiewoniu_base port.");
        return -1;
    }
    if (sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/tiewoniu_base is opened.");
    }
    else
    {
        return -1;
    }
    ros::Rate loop_rate(20);
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped transformStamped;
    ros::Time last_time, current_time;                // 记录时间
    double x_odom = 0.0, y_odom = 0.0, th_odom = 0.0; // x方向的里程计、y方向的里程计，旋转角度
    double vx = 0.0, vy = 0.0, vth = 0.0, dt = 0.0;   // 记录车辆的实时速度(vx为线速度，vth为角速度)
    double delta_x = 0.0, delta_y = 0.0, delta_th = 0.0;
    last_time = ros::Time::now();
    tf::TransformBroadcaster br;
    while (ros::ok())
    {
        // unsigned char buffer[100];
        // sp.read(buffer, 100);
        // int pos = 0;
        // for (int i = 0; i < 100; i++)
        // {
        //     if (buffer[i] == 0xAA)
        //     {
        //         pos = i;
        //         break;
        //     }
        //     // 16进制的方式打印到屏幕
        //     // std::cout << std::hex << (buffer[i] & 0xff) << " ";
        // }
        // if (buffer[pos + 0] == 0xAA && buffer[pos + 1] == 0x55 && buffer[pos + 35] == 0xCC) // 验证接收到数据包头包尾是否正确
        // {
        //     ROS_INFO("%s", "底盘程序正常运行中！！！");
        //     int16_t mach1 = Two_OCT_turn_HEX(buffer[pos + 5], buffer[pos + 6]);
        //     int16_t mach2 = Two_OCT_turn_HEX(buffer[pos + 7], buffer[pos + 8]); // mach1为右轮，mach2为左轮
        //     cout << mach1 << mach2 << endl;
        //     Limit_check(mach1, start_r, param);
        //     Limit_check(mach2, start_l, param);
        //     double d_left = ((double)(mach2 * param - start_l));
        //     double d_right = ((double)(mach1 * param - start_r));
        //     // cout<<mach2<<" "<<mach1<<endl;
        //     double dia_distance = (d_left + d_right) / 2.0; // 轮距中心行使距离
        //     double angle = ((d_right - d_left) / diameter);
        //     double delta_x = dia_distance * cos(angle);
        //     double delta_y = -dia_distance * sin(angle);
        //     x_odom += cos(th_odom) * delta_x - sin(th_odom) * delta_y;
        //     y_odom += sin(th_odom) * delta_x + cos(th_odom) * delta_y;
        //     th_odom += angle;
        //     start_l = mach2 * param;
        //     start_r = mach1 * param;
        //     start_angle = angle_pos;
        //     current_time = ros::Time::now();
        //     odom.header.stamp = current_time;
        //     dt = (current_time - last_time).toSec();
        //     vx = dia_distance / dt;
        //     vth = angle / dt;
        //     last_time = current_time;
        //     odom.header.frame_id = "/odom";
        //     // 设置位置
        //     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, th_odom); // 航向角转化为四元数
        //     odom.pose.pose.position.x = x_odom;
        //     odom.pose.pose.position.y = y_odom;
        //     odom.pose.pose.position.z = 0.0;
        //     odom.pose.pose.orientation.x = odom_quat.x;
        //     odom.pose.pose.orientation.y = odom_quat.y;
        //     odom.pose.pose.orientation.z = odom_quat.z;
        //     odom.pose.pose.orientation.w = odom_quat.w;
        //     // 设置速度
        //     odom.child_frame_id = "/base_link"; // this->robot_frame_id;
        //     odom.twist.twist.linear.x = vx;     // this->vx;线速度
        //     odom.twist.twist.linear.y = 0.0;    // this->vy;
        //     odom.twist.twist.angular.z = vth;   // this->vth;//角速度
        //     if (vx == 0 && vth == 0)
        //     {
        //         odom.pose.covariance = {1e-9, 0, 0, 0, 0, 0,
        //                                 0, 1e-3, 1e-9, 0, 0, 0,
        //                                 0, 0, 1e6, 0, 0, 0,
        //                                 0, 0, 0, 1e6, 0, 0,
        //                                 0, 0, 0, 0, 1e6, 0,
        //                                 0, 0, 0, 0, 0, 1e-9};
        //         odom.twist.covariance = {1e-9, 0, 0, 0, 0, 0,
        //                                  0, 1e-3, 1e-9, 0, 0, 0,
        //                                  0, 0, 1e6, 0, 0, 0,
        //                                  0, 0, 0, 1e6, 0, 0,
        //                                  0, 0, 0, 0, 1e6, 0,
        //                                  0, 0, 0, 0, 0, 1e-9};
        //     }
        //     else
        //     {
        //         odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,
        //                                 0, 1e-3, 0, 0, 0, 0,
        //                                 0, 0, 1e6, 0, 0, 0,
        //                                 0, 0, 0, 1e6, 0, 0,
        //                                 0, 0, 0, 0, 1e6, 0,
        //                                 0, 0, 0, 0, 0, 1e3};
        //         odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,
        //                                  0, 1e-3, 0, 0, 0, 0,
        //                                  0, 0, 1e6, 0, 0, 0,
        //                                  0, 0, 0, 1e6, 0, 0,
        //                                  0, 0, 0, 0, 1e6, 0,
        //                                  0, 0, 0, 0, 0, 1e3};
        //     }
        //     odom_pub.publish(odom);
        //     // 发布在rviz下显示的坐标
        //     transformStamped.header.stamp = ros::Time::now();
        //     transformStamped.header.frame_id = "/odom";
        //     transformStamped.child_frame_id = "/base_link";
        //     transformStamped.transform.translation.x = x_odom;
        //     transformStamped.transform.translation.y = y_odom;
        //     transformStamped.transform.translation.z = 0.0;
        //     transformStamped.transform.rotation.x = odom_quat.x;
        //     transformStamped.transform.rotation.y = odom_quat.y;
        //     transformStamped.transform.rotation.z = odom_quat.z;
        //     transformStamped.transform.rotation.w = odom_quat.w;
        //     br.sendTransform(transformStamped);
        // }
        // ros::spinOnce();
        // loop_rate.sleep();
    }
    // 关闭串口
    sp.close();
    return 0;
}