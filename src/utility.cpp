#include "utility.h"

std::queue<PointCloudMessage> pointCloudInBuf;
std::queue<PointCloudMessage> pointCloudFilteredBuf;
std::queue<PointCloudMessage> pointCloudEdgeBuf;
std::queue<PointCloudMessage> pointCloudSurfBuf;
std::queue<PointCloudMessage> pointCloudMapBuf;
std::queue<OdometryMessage> laserOdometryBuf;
std::queue<OdometryMessage> finalOdometryBuf;

std::size_t buffer_size = 5;

std::mutex pointCloudInBuf_mutex;
std::mutex pointCloudFilteredBuf_mutex;
std::mutex pointCloudEdgeBuf_mutex;
std::mutex pointCloudSurfBuf_mutex;
std::mutex pointCloudMapBuf_mutex;
std::mutex laserOdometryBuf_mutex;
std::mutex finalOdometryBuf_mutex;

std::atomic<bool> stopFlag(false);

int scan_line = 64;
double vertical_angle = 2.0;
double scan_period = 0.1;
double max_dis = 60.0;
double min_dis = 2.0;
double map_resolution = 0.4;
lidar::Lidar lidar_param;
