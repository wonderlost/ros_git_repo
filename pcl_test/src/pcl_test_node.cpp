#include <ros/ros.h>
#include "pcl_test/pcl_test_core.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_test");
    ros::NodeHandle nh;
    
    PclTestCore core(nh);

    return 0;
}
