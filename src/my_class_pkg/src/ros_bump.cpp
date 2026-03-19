#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16MultiArray.h"
#include "tf/transform_datatypes.h"
#include <cmath>
#include <limits>

// ================= 配置区 =================
const double FORWARD_SPEED = 0.3;
const double BACKUP_SPEED = -0.2;
const double BACKUP_TIME = 2.0;
const double PAUSE_TIME = 0.5;

const double TARGET_ANGLE = M_PI;       // 180度
const double SPIN_SPEED = 0.5;          // 恒速旋转 (不减速，防止停转)
const double TOLERANCE = 0.05;          // 允许误差 (约3度)

const std::string TOPIC_CMD = "/cmd_vel";
const std::string TOPIC_BUMP = "/robot/bump_sensor";
const std::string TOPIC_IMU = "/imu/data";

// ================= 全局变量 =================
enum State { S_FORWARD, S_BACKUP, S_PAUSE_1, S_SPINNING, S_PAUSE_2 };
State current_state = S_FORWARD;
ros::Time state_start_time;
ros::Publisher vel_pub;
bool collision_detected = false;

// IMU 专属变量
double last_yaw = 0.0;
double accumulated_angle = 0.0;
bool spin_ready = false; // 标记是否已完成首帧锚定

// ================= 辅助函数 =================
double normalize_angle(double angle) {
    return atan2(sin(angle), cos(angle));
}

void publishVel(double linear, double angular) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    vel_pub.publish(cmd);
}

void changeState(State new_state) {
    if (current_state == new_state) return;
    current_state = new_state;
    state_start_time = ros::Time::now();
    
    ROS_INFO("State: %d", new_state);

    if (new_state == S_SPINNING) {
        spin_ready = false; // 重置自旋标志
        accumulated_angle = 0.0;
    }
}

// ================= 回调函数 =================

// 1. 碰撞检测
void bumperCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    if (current_state != S_FORWARD) return;
    for (int val : msg->data) {
        if (val == 1) {
            if (!collision_detected) {
                ROS_WARN("Bump! Backing up...");
                collision_detected = true;
                changeState(S_BACKUP);
            }
            return;
        }
    }
}

// 2. IMU 自旋控制 (核心逻辑：参考你的示例，恒速 + 累加)
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (current_state != S_SPINNING) return;

    // 解析 Yaw
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);
    double r, p, yaw;
    tf::Matrix3x3(q).getRPY(r, p, yaw);
    yaw = normalize_angle(yaw);

    // 【首帧锚定】
    if (!spin_ready) {
        last_yaw = yaw;
        spin_ready = true;
        ROS_INFO("Spin Started. Anchor: %.2f", last_yaw);
        return; // 第一帧不发速度，等待下一帧计算差值
    }

    // 【增量累加】解决 180 度跳变问题
    double delta = normalize_angle(yaw - last_yaw);
    accumulated_angle += delta;
    last_yaw = yaw;

    // 【判断停止】
    if (fabs(accumulated_angle) >= (TARGET_ANGLE - TOLERANCE)) {
        ROS_INFO("Spin Done. Total: %.2f rad", fabs(accumulated_angle));
        publishVel(0, 0);
        changeState(S_PAUSE_2);
        return;
    }

    // 【恒速输出】只要没到，就一直给满速度 (方向：逆时针为正)
    publishVel(0, SPIN_SPEED);
}

// ================= 主循环 =================
void timerCallback(const ros::TimerEvent&) {
    double elapsed = (ros::Time::now() - state_start_time).toSec();

    switch (current_state) {
        case S_FORWARD:
            publishVel(FORWARD_SPEED, 0);
            break;
        case S_BACKUP:
            publishVel(BACKUP_SPEED, 0);
            if (elapsed >= BACKUP_TIME) changeState(S_PAUSE_1);
            break;
        case S_PAUSE_1:
            publishVel(0, 0);
            if (elapsed >= PAUSE_TIME) changeState(S_SPINNING);
            break;
        case S_SPINNING:
            // 【关键】此处完全不发速度，由 imuCallback 全权负责
            // 仅做一个简单的超时保护，防止 IMU 彻底坏掉时死锁
            if (elapsed > 30.0) {
                ROS_ERROR("Spin Timeout");
                publishVel(0, 0);
                changeState(S_PAUSE_2);
            }
            break;
        case S_PAUSE_2:
            publishVel(0, 0);
            if (elapsed >= PAUSE_TIME) {
                collision_detected = false;
                changeState(S_FORWARD);
            }
            break;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_bump_spin");
    ros::NodeHandle nh;

    vel_pub = nh.advertise<geometry_msgs::Twist>(TOPIC_CMD, 10);
    ros::Subscriber sub_bump = nh.subscribe(TOPIC_BUMP, 10, bumperCallback);
    ros::Subscriber sub_imu = nh.subscribe(TOPIC_IMU, 10, imuCallback);
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), timerCallback);

    ROS_INFO("Ready. Simple Bump & Spin (No Deceleration).");
    ros::spin();
    return 0;
}