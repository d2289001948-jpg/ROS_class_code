#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

// 全局变量：记录状态、初始位置、当前偏航角
bool init_pose_flag = false;  // 是否获取初始位置
double init_x, init_y;        // 初始坐标（正方形起点）
double current_yaw = 0.0;     // 当前偏航角（弧度）
int move_step = 1;            // 1=直走 2=旋转
bool step_finish = false;     // 单步动作是否完成
int square_count = 0;         // 完成的“直走+旋转”次数（初值0，到4停止）
double rotate_init_yaw = 0.0; // 旋转步骤的初始角度（全局变量，避免static坑）

// 里程计回调函数：获取实时位置和偏航角
void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    // 1. 获取机器人当前x/y坐标
    double current_x = odom_msg->pose.pose.position.x;
    double current_y = odom_msg->pose.pose.position.y;

    // 2. 将四元数转为偏航角（绕z轴，范围-π~π，正为逆时针）
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quat);
    double roll, pitch;
    tf::Matrix3x3(quat).getRPY(roll, pitch, current_yaw);

    // 3. 初始化：记录正方形起点坐标
    if (!init_pose_flag)
    {
        init_x = current_x;
        init_y = current_y;
        init_pose_flag = true;
        ROS_INFO("初始化起点：x=%.2f, y=%.2f", init_x, init_y);
        return;
    }

    // 4. 闭环判断：根据当前步骤，判断是否完成动作
    switch (move_step)
    {
        // 步骤1：直走1米
        case 1:
        {
            double distance = sqrt(pow(current_x - init_x, 2) + pow(current_y - init_y, 2));
            ROS_INFO_THROTTLE(0.5, "当前直走距离：%.2f米", distance); // 0.5秒打印一次，不刷屏
            
            // 直走1米完成（加0.05容错，避免里程计误差导致卡壳）
            if (distance >= 0.95)  
            {
                step_finish = true;
                ROS_INFO("✅ 直走1米完成！实际距离：%.2f米", distance);
                // 更新下一次直走的起点
                init_x = current_x;
                init_y = current_y;
                // 记录旋转的初始角度（切换到旋转前先存当前角度）
                rotate_init_yaw = current_yaw;
            }
            break;
        }

        // 步骤2：原地逆时针转90度（π/2≈1.57弧度）
        case 2:
        {
            // 计算角度差（处理-π~π的循环，比如从3.0弧度转到-2.0弧度，实际差1.28）
            double yaw_diff = fabs(current_yaw - rotate_init_yaw);
            // 处理跨π的情况（比如从3.0→-3.0，实际差0.28，不是6.0）
            if (yaw_diff > M_PI) {
                yaw_diff = 2 * M_PI - yaw_diff;
            }
            
            ROS_INFO_THROTTLE(0.5, "当前旋转角度差：%.2f弧度（目标1.57）", yaw_diff);
            
            // 旋转90度完成（加0.05容错）
            if (yaw_diff >= 1.52)  
            {
                step_finish = true;
                ROS_INFO("✅ 旋转90度完成！实际角度差：%.2f弧度", yaw_diff);
            }
            break;
        }

        default:
            break;
    }
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "odom_square_node");
    ros::NodeHandle nh;

    // 订阅里程计话题（队列大小设20，避免数据丢失）
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 20, odom_callback);
    // 发布速度指令到/cmd_vel
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 速度消息初始化：默认停止
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;

    ros::Rate loop_rate(50);  // 循环频率50Hz
    ROS_INFO("🚗 开始执行1×1正方形闭环运动！");

    while (ros::ok() && square_count < 4)  // count到4就停止
    {
        // 未获取初始位置，先停止等待
        if (!init_pose_flag)
        {
            cmd_pub.publish(cmd_vel);
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        // 单步动作完成，切换步骤+更新count
        if (step_finish)
        {
            // 先停止机器人
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_pub.publish(cmd_vel);
            ros::Duration(0.8).sleep();  // 停顿0.8秒，动作更平稳
            step_finish = false;

            // 步骤切换+count计数规则
            if (move_step == 1) 
            {
                // 直走完成 → 切换到旋转
                move_step = 2;
                ROS_INFO("🔄 准备旋转90度，当前count：%d/4", square_count);
            } 
            else if (move_step == 2) 
            {
                // 旋转完成 → 切换到直走，count+1
                move_step = 1;
                square_count++;  
                ROS_INFO("📌 旋转完成！count+1，当前count：%d/4", square_count);
            }
        }

        // 执行当前步骤的速度指令
        if (move_step == 1)  // 直走步骤：线速度0.2，角速度0
        {
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = 0.0;
            ROS_INFO_THROTTLE(1, "🔴 正在直走1米...");
        }
        else if (move_step == 2)  // 旋转步骤：线速度0，角速度0.3（加快旋转，避免卡壳）
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.3;
            ROS_INFO_THROTTLE(1, "🟡 正在旋转90度...");
        }

        // 发布速度指令+执行回调
        cmd_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // count=4，运动完成，最终停止
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_pub.publish(cmd_vel);
    ROS_INFO("🎉 1×1正方形运动完成！最终count值：%d，机器人已停止", square_count);

    return 0;
}