//
// Created by gx on 17-9-12.
//


#include <ros/ros.h>

#include <geometry_msgs/WrenchStamped.h>

#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ethdaq_data");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::WrenchStamped>("ethdaq_data", 1);

    ros::Rate loop_rate(250);

    int count = 0;
    while (ros::ok())
    {

        geometry_msgs::WrenchStamped data;
        data.wrench.force.x=0;
        data.wrench.force.y=0;
        data.wrench.force.z=0;
        data.wrench.torque.x=0;
        data.wrench.torque.y=0;
        data.wrench.torque.z=0;
        ROS_INFO_STREAM("force_z "<< data.wrench.force.z);
        chatter_pub.publish(data);
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
