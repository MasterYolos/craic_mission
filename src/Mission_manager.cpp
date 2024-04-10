#include "ros/ros.h"
#include <Px4sp_msgs.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_manager");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<craic_mission::Px4sp_msgs>("px4_command",10);

    ros::Rate loop_rate(10); // 10Hz

    craic_mission::Px4sp_msgs msg;
    msg.x = 2.0;
    msg.y = 2.0;
    msg.z = 2.0;
    msg.yaw = 0.0;

    while (ros::ok())
    {
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}