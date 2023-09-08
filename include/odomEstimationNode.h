#pragma once

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//pcl lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "utility.h"
#include "odomEstimationClass.h"

class odomEstimationNode
{
public:

    void odom_estimation();

    bool is_odom_inited = false;

    OdomEstimationClass odomEstimation;

    PointCloudMessage pointcloud_edge_in_msg;
    PointCloudMessage pointcloud_surf_in_msg;

};