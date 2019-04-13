//I/O Stream
//std::cout
#include <iostream>

#include <string>

// ROS
//ros::init(), ros::NodeHandle, ros::ok(), ros::spinOnce()
#include "ros/ros.h"
#include "laser_based_rrt_star.h"


using namespace std;


int main(int argc, char **argv)
{
    //Init
    ros::init(argc, argv, "laser_based_rrt_star_node"); //Say to ROS the name of the node and the parameters
    ros::NodeHandle n; //Este nodo admite argumentos!!

    //std::string node_name=ros::this_node::getName();
    //cout<<"node name="<<node_name<<endl;
//    ros::Rate r(100);
    //Class definition

    LASER_BASED_RRT_STAR laser_based_rrt_star;
    //Open!

    laser_based_rrt_star.open(n);


    //Loop -> Ashyncronous Module
    while(ros::ok())
    {

        laser_based_rrt_star.Publish_P();
        laser_based_rrt_star.reset();
        laser_based_rrt_star.path_plan_start = true;
        ros::spin();
//        r.sleep();
    }

    return 1;
}
