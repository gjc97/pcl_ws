//
// Created by adam on 18-9-21.
//

#include "pcl_test_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_test");

    ros::NodeHandle nh;
    //实例化一个类，core(nh)通过传入一个节点句柄来调用构造函数，
    PclTestCore core(nh);
    // core.Spin();
    return 0;
}