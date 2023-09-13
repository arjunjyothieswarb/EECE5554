#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_poly");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",100);
    ros::Rate loop(5);

    geometry_msgs::Twist move, rot;

    move.angular.z = 0.0;
    move.linear.x = 2.0;

    rot.angular.z = 2.0;
    rot.linear.x = 0.0;

    int trig=0;

    while(ros::ok())
    {
        trig = trig % 2;
        
        if(trig)pub.publish(move);
        else pub.publish(rot);

        trig++;

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}