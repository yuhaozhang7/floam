#pragma once

#include <iostream>
#include <queue>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar.h"

struct PointCloudMessage;
struct Odometry;

extern std::queue<PointCloudMessage> pointCloudInBuf;
extern std::queue<PointCloudMessage> pointCloudFilteredBuf;
extern std::queue<PointCloudMessage> pointCloudEdgeBuf;
extern std::queue<PointCloudMessage> pointCloudSurfBuf;

extern std::mutex pointCloudInBuf_mutex;
extern std::mutex pointCloudFilteredBuf_mutex;
extern std::mutex pointCloudEdgeBuf_mutex;
extern std::mutex pointCloudSurfBuf_mutex;

extern lidar::Lidar lidar_param;

extern int buffer_size;


struct Odometry
{   
    std::string topic;
    std::string frame_id;
    double timestamp;
    double orientationX;
    double orientationY;
    double orientationZ;
    double orientationW;
    double positionX;
    double positionY;
    double positionZ;
    double angularX;
    double angularY;
    double angularZ;
    double linearX;
    double linearY;
    double linearZ;
};

struct PointCloudMessage
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  double timestamp;
  std::string frame_id;
};
