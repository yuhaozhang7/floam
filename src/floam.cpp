#include "floam.h"

bool FLOAM::Init() {
    // Initialize components
    LP.laserProcessing.init(lidar_param);
    OE.odomEstimation.init(lidar_param, map_resolution);
    LM.laserMapping.init(map_resolution);

    // Start and detach threads
    laser_processing_thread = std::thread(&laserProcessingNode::laser_processing, &LP);
    laser_processing_thread.detach();

    odom_estimation_thread = std::thread(&odomEstimationNode::odom_estimation, &OE);
    odom_estimation_thread.detach();

    laser_mapping_thread = std::thread(&laserMappingNode::laser_mapping, &LM);
    laser_mapping_thread.detach();

    return true;
}


void FLOAM::AdjustInput(const PointCloudMessage &pointCloudInMsg) {

    while (true) {
        if (pointCloudInBuf.size() < buffer_size) {
            pointCloudInBuf_mutex.lock();
            pointCloudInBuf.push(pointCloudInMsg);
            pointCloudInBuf_mutex.unlock();
            break; // break out of the while loop
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep for a short duration before checking again
    }

}


std::pair<OdometryMessage, PointCloudMessage> FLOAM::Process() {
    auto output_pair = getOutput();

    OdometryMessage pose_msg = output_pair.first;
    PointCloudMessage cloud_msg = output_pair.second;

    if (pose_msg.timestamp == 0.0) {
        std::cout << "timestamp is zero, program stop" << std::endl;
        stopFlag.store(true);
    }

    return output_pair;
}


std::pair<OdometryMessage, PointCloudMessage> FLOAM::getOutput() {

    auto start_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time);

    OdometryMessage output_odom_msg;
    PointCloudMessage output_map_msg;

    output_odom_msg.timestamp = 0.0;
    output_map_msg.timestamp = 0.0;

    while (elapsed_time.count() < 5000) {

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
            std::cout << "output pose and point cloud are not synchronized !" << std::endl;
            output_odom_msg.timestamp = 0.0;
            output_map_msg.timestamp = 0.0;
            return std::make_pair(output_odom_msg, output_map_msg);
        }
        
    }
    std::cout << "output buffer is empty for over 5s, set timestamp to zero" << std::endl;
    return std::make_pair(output_odom_msg, output_map_msg);
    // throw std::runtime_error("Timeout waiting for buffers!");
}
