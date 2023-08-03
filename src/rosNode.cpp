#include "rosNode.hpp"
int main(int argc , char **argv)
{
    ros::init(argc,argv,"yolov8_node");
    ros::NodeHandle n;
    if(argc == 2)
    {
        Segment segment(n,argv[1]);
        ros::spin();
    } else
    {
        ROS_ERROR("Please give paraments path!");
    }

    return 0;
}

