#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Twist.h>

ros::Publisher vel_pub;
bool bump_flag = false;

void bumpCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    for(int i=0; i<msg->data.size(); i++)
    {
        if(msg->data[i] == 1)
        {
            bump_flag = true;
            break;
        }
        else
        {
            bump_flag = false;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bump_avoid_node");
    ros::NodeHandle n;

    ros::Subscriber bump_sub = n.subscribe("/robot/bump_sensor", 10, bumpCallback);
    vel_pub = n.advertise<geometry_msgs/Twist>("/cmd_vel", 10);

    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel_msg;

    while(ros::ok())
    {
        if(bump_flag)
        {
            vel_msg.linear.x = -0.2;
            vel_msg.angular.z = 0.5;
            vel_pub.publish(vel_msg);
            sleep(1);

            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
            vel_pub.publish(vel_msg);
            bump_flag = false;
        }
        else
        {
            vel_msg.linear.x = 0.2;
            vel_msg.angular.z = 0;
            vel_pub.publish(vel_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
