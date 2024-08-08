#include <ros/ros.h>
#include <modbus/modbus.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

class DS2024_controller
{
public:
    DS2024_controller();
    void cmdVel_callback(const geometry_msgs::Twist::ConstPtr &msg); // 速度订阅函数
    void enable_motor();                                             // 轴使能函数
    void set_rpm(int16_t rpm_left, int16_t rpm_right);
    void poseUpdate();
    ros::NodeHandle node;
    ros::Subscriber sub_cmdVel = node.subscribe("cmd_vel", 10, &DS2024_controller::cmdVel_callback, this);
    ros::Publisher pub_odom = node.advertise<nav_msgs::Odometry>("odom", 10);

private:
    string serial_name="/dev/tiewoniu_base";
    geometry_msgs::Twist vel;
    const int32_t L_ENABLE = 0x3100;      // 左轮使能地址；
    const int32_t R_ENABLE = 0x2100;      // 右轮使能地址；
    const int32_t L_CMD_RPM = 0x3318;     // 左轮速度地址
    const int32_t R_CMD_RPM = 0x2318;     // 右轮速度地址
    const int32_t L_ENCODER_RPM = 0x5104; // 左轮编码器地址
    const int32_t R_ENCODER_RPM = 0x5004; // 右轮编码器地址
    const int16_t ENABLE = 1;
    modbus_t *ctx;
    nav_msgs::Odometry odom;          // 里程计模型
    tf2_ros::TransformBroadcaster br; // 发布tf树
    geometry_msgs::TransformStamped transformStamped;
    ros::Time current_time; // 记录时间
    ros::Time last_time = ros::Time::now();
    float base = 0.358;                // 两车轮之间的距离
    float rps_2_rpm = 9.54929658551;  //(60/(2*pi))
    float wheel = 0.0845;             // 车轮半径
    double param = 0.000094808778296; // 每走1个脉冲信号车辆行走0.000094808778295m
    u_int16_t Last_L = 0;             // 机器人初始位姿定义,xy坐标，angle角度
    u_int16_t Last_R = 0;
    double x_odom = 0.0, y_odom = 0.0, th_odom = 0.0; // x方向的里程计、y方向的里程计，旋转角度
    double vx = 0.0, vy = 0.0, vth = 0.0, dt = 0.0;   // 记录车辆的实时速度(vx为线速度，vth为角速度)
    int16_t int16Dec_to_int16Hex(int16_t data)        // 将10进制数转换为16进制数，并分为高低位存储；
    {
        uint8_t low_byte;
        uint8_t high_byte;
        int16_t all_bytes;
        low_byte = (data & 0x00FF);
        high_byte = (data & 0xFF00) >> 8;
        all_bytes = (high_byte << 8) | low_byte;
        return all_bytes;
    }
    void Limit_check(int16_t &mach)
    {
        if (mach < 0 && abs(mach) > 5000)
        {
            mach = 5600 + mach;
        }
        else if (mach > 0 && abs(mach) > 5000)
        {
            mach = mach - 5600;
        }
        else
        {
            mach = mach;
        }
    }
};
void DS2024_controller::poseUpdate()
{
    u_int16_t tab_reg_L[1];
    modbus_read_registers(ctx, L_ENCODER_RPM, 1, tab_reg_L);
    u_int16_t tab_reg_R[1];
    modbus_read_registers(ctx, R_ENCODER_RPM, 1, tab_reg_R);
    // cout << tab_reg_L[0] << " " << tab_reg_R[0] << endl;
    int16_t R_M = (tab_reg_R[0] - Last_R);
    int16_t L_M = -(tab_reg_L[0] - Last_L);
    Limit_check(R_M);
    Limit_check(L_M);
    //cout << L_M << " " << R_M << endl;
    double d_left = L_M * param;
    double d_right = R_M * param;
    double dia_distance = (d_left + d_right) / 2.0; // 轮距中心行使距离
    double angle =-((d_left - d_right) / base);
    double delta_x = dia_distance * cos(angle);
    double delta_y = -dia_distance * sin(angle);
    x_odom += cos(th_odom) * delta_x - sin(th_odom) * delta_y;
    y_odom += sin(th_odom) * delta_x + cos(th_odom) * delta_y;
    th_odom += angle;
    Last_L = tab_reg_L[0];
    Last_R = tab_reg_R[0];
    current_time = ros::Time::now();
    odom.header.stamp = current_time;
    dt = (current_time - last_time).toSec();
    vx = dia_distance / dt;
    vth = angle / dt;
    last_time = current_time;
    odom.header.frame_id = "odom";
    // 设置位置
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, th_odom); // 航向角转化为四元数；
    odom.pose.pose.position.x = x_odom;
    odom.pose.pose.position.y = y_odom;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();
    // 设置速度
    odom.child_frame_id = "base_link"; // this->robot_frame_id;
    odom.twist.twist.linear.x = vx;     // this->vx;线速度
    odom.twist.twist.linear.y = 0.0;    // this->vy;
    odom.twist.twist.angular.z = vth;   // this->vth;//角速度
    if (vx == 0 && vth == 0)
    {
        odom.pose.covariance = {1e-9, 0, 0, 0, 0, 0,
                                0, 1e-3, 1e-9, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e-9};
        odom.twist.covariance = {1e-9, 0, 0, 0, 0, 0,
                                 0, 1e-3, 1e-9, 0, 0, 0,
                                 0, 0, 1e6, 0, 0, 0,
                                 0, 0, 0, 1e6, 0, 0,
                                 0, 0, 0, 0, 1e6, 0,
                                 0, 0, 0, 0, 0, 1e-9};
    }
    else
    {
        odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,
                                0, 1e-3, 0, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e3};
        odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,
                                 0, 1e-3, 0, 0, 0, 0,
                                 0, 0, 1e6, 0, 0, 0,
                                 0, 0, 0, 1e6, 0, 0,
                                 0, 0, 0, 0, 1e6, 0,
                                 0, 0, 0, 0, 0, 1e3};
    }
    pub_odom.publish(odom);
    // 发布在rviz下显示的坐标
    //transformStamped.header.stamp = ros::Time::now();
    //transformStamped.header.frame_id ="odom";
    //transformStamped.child_frame_id = "base_link";
    //transformStamped.transform.translation.x = x_odom;
    //transformStamped.transform.translation.y = y_odom;
    //transformStamped.transform.translation.z = 0.0;
    //transformStamped.transform.rotation.x = odom_quat.x();
    //transformStamped.transform.rotation.y = odom_quat.y();
    //transformStamped.transform.rotation.z = odom_quat.z();
    //transformStamped.transform.rotation.w = odom_quat.w();
    //br.sendTransform(transformStamped);
}

void DS2024_controller::enable_motor()
{
    if (modbus_write_register(ctx, L_ENABLE, ENABLE) != 1)
    {
        ROS_FATAL("左轴使能失败！");
    }
    if (modbus_write_register(ctx, R_ENABLE, ENABLE) != 1)
    {
        ROS_FATAL("右轴使能失败！");
    }
}

void DS2024_controller::cmdVel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    vel.linear.x = -msg->linear.x;
    vel.angular.z = -msg->angular.z;
    int command_left, command_right;
    command_left = (vel.linear.x - (vel.angular.z * base / 2)) / wheel * rps_2_rpm;
    command_right = (vel.linear.x + (vel.angular.z * base / 2)) / wheel * rps_2_rpm; // RPS-> RPM (1/(2*pi))*60
    set_rpm(-command_left, command_right);
}

void DS2024_controller::set_rpm(int16_t rpm_left, int16_t rpm_right)
{
    int16_t rpm[2] = {rpm_left, rpm_right};
    if (rpm[0] > 6000)
    {
        rpm[0] = 6000;
    }
    else if (rpm[0] < -6000)
    {
        rpm[0] = -6000;
    }

    if (rpm[1] > 6000)
    {
        rpm[1] = 6000;
    }
    else if (rpm[1] < -6000)
    {
        rpm[1] = -6000;
    }
    u_int16_t rpm_conmand[2];
    rpm_conmand[0] = int16Dec_to_int16Hex(rpm[0]);
    rpm_conmand[1] = int16Dec_to_int16Hex(rpm[1]);
    modbus_write_register(ctx, R_CMD_RPM, rpm_conmand[1]);
    modbus_write_register(ctx, L_CMD_RPM, rpm_conmand[0]);
}

DS2024_controller::DS2024_controller() // 类构造函数，负责初始化modbus通讯接口；
{
    ROS_INFO("电机驱动正在连接...");
    int device_ID = 1;
    ctx = modbus_new_rtu("/dev/ttyTHS1", 115200, 'N', 8, 1);
    if (ctx == NULL)
    {
        ROS_FATAL("无法创建libmodbus上下文 ...");
        return;
    }
    modbus_set_slave(ctx, device_ID);
    if (modbus_connect(ctx) == -1)
    {
        ROS_FATAL("连接失败: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return;
    }
    else
    {
        ROS_INFO("电机驱动连接成功！");
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "DS2024_motor_driver");
    DS2024_controller driver;
    driver.enable_motor();
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        driver.poseUpdate();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
