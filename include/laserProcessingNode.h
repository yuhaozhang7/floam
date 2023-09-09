#pragma once

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"
#include "utility.h"

class laserProcessingNode 
{ 
public:

    void laser_processing();

    PointCloudMessage pointcloud_in_msg;
    // PointCloudMessage pointcloud_filtered_msg;
    // PointCloudMessage pointcloud_edge_msg;
    // PointCloudMessage pointcloud_surf_msg;

    LaserProcessingClass laserProcessing;

};
