#include "utility.h"

std::queue<PointCloudMessage> pointCloudInBuf;
std::queue<PointCloudMessage> pointCloudFilteredBuf;
std::queue<PointCloudMessage> pointCloudEdgeBuf;
std::queue<PointCloudMessage> pointCloudSurfBuf;
std::queue<PointCloudMessage> pointCloudMapBuf;
std::queue<OdometryMessage> laserOdometryBuf;
std::queue<OdometryMessage> finalOdometryBuf;

std::mutex pointCloudInBuf_mutex;
std::mutex pointCloudFilteredBuf_mutex;
std::mutex pointCloudEdgeBuf_mutex;
std::mutex pointCloudSurfBuf_mutex;
std::mutex pointCloudMapBuf_mutex;
std::mutex laserOdometryBuf_mutex;
std::mutex finalOdometryBuf_mutex;

std::atomic<bool> stopFlag(false);

lidar::Lidar lidar_param;

std::size_t buffer_size = 5;

std::pair<OdometryMessage, PointCloudMessage> getMap() {

    auto start_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time);

    OdometryMessage output_odom_msg;
    PointCloudMessage output_map_msg;

    output_odom_msg.timestamp = 0.0;
    output_map_msg.timestamp = 0.0;

    while (elapsed_time.count() < 500) {

        finalOdometryBuf_mutex.lock();
        pointCloudMapBuf_mutex.lock();

        if (!pointCloudMapBuf.empty() && !finalOdometryBuf.empty()) {

            output_odom_msg = finalOdometryBuf.front();
            output_map_msg = pointCloudMapBuf.front();
            finalOdometryBuf_mutex.unlock();
            pointCloudMapBuf_mutex.unlock();

        } else {

            finalOdometryBuf_mutex.unlock();
            pointCloudMapBuf_mutex.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Sleep for 10ms before checking again
            elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time);

            continue;
        }

        double difference = std::abs(output_odom_msg.timestamp - output_map_msg.timestamp);
        if (difference < 0.5*lidar_param.scan_period) {
            finalOdometryBuf_mutex.lock();
            pointCloudMapBuf_mutex.lock();
            finalOdometryBuf.pop();
            pointCloudMapBuf.pop();
            finalOdometryBuf_mutex.unlock();
            pointCloudMapBuf_mutex.unlock();
            return std::make_pair(output_odom_msg, output_map_msg);
        } else {
            output_odom_msg.timestamp = 0.0;
            output_map_msg.timestamp = 0.0;
            return std::make_pair(output_odom_msg, output_map_msg);
        }
        
    }
    return std::make_pair(output_odom_msg, output_map_msg);
    // throw std::runtime_error("Timeout waiting for buffers!");
}
