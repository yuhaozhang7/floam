#include "utility.h"

std::queue<PointCloudMessage> pointCloudInBuf;
std::queue<PointCloudMessage> pointCloudFilteredBuf;
std::queue<PointCloudMessage> pointCloudEdgeBuf;
std::queue<PointCloudMessage> pointCloudSurfBuf;

std::mutex pointCloudInBuf_mutex;
std::mutex pointCloudFilteredBuf_mutex;
std::mutex pointCloudEdgeBuf_mutex;
std::mutex pointCloudSurfBuf_mutex;

lidar::Lidar lidar_param;

int buffer_size = 5;
