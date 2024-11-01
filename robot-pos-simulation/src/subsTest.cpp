#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <vector>
#include <turtlesim/Pose.h>

using namespace ros;
using namespace std;

void callback(const turtlesim::Pose msg){

    ROS_INFO("x=[%.2f] y=[%.2f]\n", msg.x, msg.y);
}

int main(int argc, char** argv){
    init(argc, argv, "subsTest");
    NodeHandle nh;
    Subscriber sub = nh.subscribe("/turtle1/pose",1, callback);
    spin();
    return 0;
}