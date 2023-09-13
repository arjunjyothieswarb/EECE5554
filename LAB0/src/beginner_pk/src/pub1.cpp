#include "ros/ros.h"
#include "std_msgs/Int16.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int16>("chatter",100);
    ros::Rate loop(10);

    std_msgs::Int16 val;
    val.data = 10;

    while(ros::ok())
    {
        pub.publish(val);

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}