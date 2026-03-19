#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include <cmath>
#include <limits>

// ================= 1. 参数配置 =================
namespace Config {
    // 运动参数
    const double FORWARD_SPEED = 0.3;
    const double BACKUP_SPEED = -0.3;       // 稍微加大后退力度，快速脱离
    const double BACKUP_TIME = 2.0;
    const double PAUSE_TIME = 0.5;

    // TOF 避障参数
    const double OBSTACLE_DIST = 0.4;       // 严格保持 0.4m
    const std::string TOPIC_TOF = "/us/tof2"; 

    // 自旋参数
    const double TARGET_ANGLE = M_PI;       
    const double SPIN_SPEED = 0.5;          
    const double TOLERANCE = 0.05;          

    // 话题
    const std::string TOPIC_CMD = "/cmd_vel";
    const std::string TOPIC_IMU = "/imu/data";
}

// ================= 2. 状态定义 =================
enum RobotState {
    STATE_FORWARD,
    STATE_BACKUP,
    STATE_PAUSE_1,
    STATE_SPINNING,
    STATE_PAUSE_2
};

RobotState current_state = STATE_FORWARD;
ros::Time state_start_time;
ros::Publisher vel_pub;
bool obstacle_detected = false; 

// ================= 3. IMU 自旋专用变量 =================
double last_yaw = 0.0;
double accumulated_angle = 0.0;
bool spin_ready = false; 

// ================= 4. 辅助函数 =================
double normalize_angle(double angle) {
    return atan2(sin(angle), cos(angle));
}

void publishVel(double linear, double angular) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    vel_pub.publish(cmd);
}

void changeState(RobotState new_state) {
    if (current_state == new_state) return;

    ROS_INFO("State Change: %d -> %d", current_state, new_state);
    current_state = new_state;
    state_start_time = ros::Time::now();

    if (new_state == STATE_SPINNING) {
        spin_ready = false;
        accumulated_angle = 0.0;
        ROS_INFO("Preparing to spin 180 degrees...");
    }
}

// ================= 5. 回调函数 =================

// 【关键修改】TOF 回调：加入紧急制动逻辑
void tofCallback(const sensor_msgs::Range::ConstPtr& msg) {
    // 数据有效性检查
    if (!std::isfinite(msg->range)) return;

    // 只有在前进状态下才需要紧急制动
    if (current_state == STATE_FORWARD) {
        if (msg->range < Config::OBSTACLE_DIST) {
            // 【紧急制动】：发现危险，立刻把速度清零！不管主循环在干什么
            // 这一步能防止在状态切换的几十毫秒内继续撞击
            publishVel(0.0, 0.0); 
            
            if (!obstacle_detected) {
                ROS_WARN("EMERGENCY STOP! Dist: %.2f m < %.2f m", msg->range, Config::OBSTACLE_DIST);
                obstacle_detected = true;
                // 注意：这里不直接 changeState，因为回调里改状态不安全
                // 我们依靠主循环检测到 obstacle_detected 为 true 来切换状态
            }
        }
    }
}

// IMU 回调 (保持不变)
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (current_state != STATE_SPINNING) return;

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);
    double r, p, yaw;
    tf::Matrix3x3(q).getRPY(r, p, yaw);
    yaw = normalize_angle(yaw);

    if (!spin_ready) {
        last_yaw = yaw;
        spin_ready = true;
        return; 
    }

    double delta = normalize_angle(yaw - last_yaw);
    accumulated_angle += delta;
    last_yaw = yaw;

    if (fabs(accumulated_angle) >= (Config::TARGET_ANGLE - Config::TOLERANCE)) {
        ROS_INFO_STREAM("Spin Complete. Total: " << fabs(accumulated_angle));
        publishVel(0.0, 0.0);
        changeState(STATE_PAUSE_2);
        return;
    }

    publishVel(0.0, Config::SPIN_SPEED);
}

// ================= 6. 主状态机循环 =================
void stateLoop(const ros::TimerEvent&) {
    double elapsed = (ros::Time::now() - state_start_time).toSec();

    // 【关键修改】在主循环优先检查避障标志
    // 如果 TOF 回调已经触发了 obstacle_detected，且当前还在前进，立即切换状态
    if (obstacle_detected && current_state == STATE_FORWARD) {
        changeState(STATE_BACKUP);
        return; // 本周期不再执行后续逻辑
    }

    switch (current_state) {
        case STATE_FORWARD:
            // 正常前进
            publishVel(Config::FORWARD_SPEED, 0.0);
            break;

        case STATE_BACKUP:
            publishVel(Config::BACKUP_SPEED, 0.0);
            if (elapsed >= Config::BACKUP_TIME) {
                changeState(STATE_PAUSE_1);
            }
            break;

        case STATE_PAUSE_1:
            publishVel(0.0, 0.0);
            if (elapsed >= Config::PAUSE_TIME) {
                changeState(STATE_SPINNING);
            }
            break;

        case STATE_SPINNING:
            // 速度由 imuCallback 控制
            if (elapsed > 30.0) {
                ROS_ERROR("Spin Timeout!");
                publishVel(0.0, 0.0);
                changeState(STATE_PAUSE_2);
            }
            break;

        case STATE_PAUSE_2:
            publishVel(0.0, 0.0);
            if (elapsed >= Config::PAUSE_TIME) {
                obstacle_detected = false;
                changeState(STATE_FORWARD);
            }
            break;
    }
}

// ================= 7. Main =================
int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_sonic_node");
    ros::NodeHandle nh;

    vel_pub = nh.advertise<geometry_msgs::Twist>(Config::TOPIC_CMD, 10);

    // 订阅 TOF (队列长度设为 1，保证拿到最新数据)
    ros::Subscriber sub_tof = nh.subscribe<sensor_msgs::Range>(
        Config::TOPIC_TOF, 1, tofCallback);

    // 订阅 IMU
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>(
        Config::TOPIC_IMU, 10, imuCallback);

    // 状态机定时器 (频率提高到 20Hz，响应更快)
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), stateLoop);

    ROS_INFO("==========================================");
    ROS_INFO("  TOF Avoidance (Emergency Brake Enabled)");
    ROS_INFO("  Trigger: < %.2f m (Hard Stop)", Config::OBSTACLE_DIST);
    ROS_INFO("==========================================");

    ros::spin();
    return 0;
}