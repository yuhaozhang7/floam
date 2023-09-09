#pragma once

#include <iostream>
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar.h"

struct PointCloudMessage;
struct OdometryMessage;

extern std::queue<PointCloudMessage> pointCloudInBuf;
extern std::queue<PointCloudMessage> pointCloudFilteredBuf;
extern std::queue<PointCloudMessage> pointCloudEdgeBuf;
extern std::queue<PointCloudMessage> pointCloudSurfBuf;
extern std::queue<PointCloudMessage> pointCloudMapBuf;
extern std::queue<OdometryMessage> laserOdometryBuf;
extern std::queue<OdometryMessage> finalOdometryBuf;

extern std::mutex pointCloudInBuf_mutex;
extern std::mutex pointCloudFilteredBuf_mutex;
extern std::mutex pointCloudEdgeBuf_mutex;
extern std::mutex pointCloudSurfBuf_mutex;
extern std::mutex pointCloudMapBuf_mutex;
extern std::mutex laserOdometryBuf_mutex;
extern std::mutex finalOdometryBuf_mutex;

extern std::atomic<bool> stopFlag;

extern lidar::Lidar lidar_param;

extern std::size_t buffer_size;

std::pair<OdometryMessage, PointCloudMessage> getMap();

struct OdometryMessage
{   
    std::string topic;
    std::string frame_id;
    std::string child_frame_id;
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
