//
// Created by adam on 18-10-8.
//

#include "euclidean_cluster_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "euclidean_cluster");

    ros::NodeHandle nh;

    EuClusterCore core(nh);
    return 0;
}