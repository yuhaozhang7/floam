// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "laserMappingNode.h"

void laserMappingNode::laser_mapping(){
    while(!stopFlag.load()){
        if(!laserOdometryBuf.empty() && !pointCloudFilteredBuf.empty()){

            //read data
            laserOdometryBuf_mutex.lock();
            if(!laserOdometryBuf.empty() && laserOdometryBuf.front().timestamp < pointCloudFilteredBuf.front().timestamp-0.5*lidar_param.scan_period){
                laserOdometryBuf.pop();
                std::cout << "time stamp unaligned with path final, pls check your data --> laser mapping node" << std::endl;
                laserOdometryBuf_mutex.unlock();
                continue;
            }

            pointCloudFilteredBuf_mutex.lock();
            if(!pointCloudFilteredBuf.empty() && pointCloudFilteredBuf.front().timestamp < laserOdometryBuf.front().timestamp-0.5*lidar_param.scan_period){
                std::cout << "time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node" << std::endl; 
                pointCloudFilteredBuf.pop();
                pointCloudFilteredBuf_mutex.unlock();
                continue;         
            }

            //if time aligned
            double pointcloud_time = laserOdometryBuf.front().timestamp;

            OdometryMessage laserodom_in_msg = laserOdometryBuf.front();
            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(laserodom_in_msg.orientationW, laserodom_in_msg.orientationX, laserodom_in_msg.orientationY, laserodom_in_msg.orientationZ));  
            current_pose.pretranslate(Eigen::Vector3d(laserodom_in_msg.positionX, laserodom_in_msg.positionY, laserodom_in_msg.positionZ));

            PointCloudMessage poincloud_filter_in_msg = pointCloudFilteredBuf.front();

            laserOdometryBuf.pop();
            pointCloudFilteredBuf.pop();

            laserOdometryBuf_mutex.unlock();
            pointCloudFilteredBuf_mutex.unlock();

            laserMapping.updateCurrentPointsToMap(poincloud_filter_in_msg.cloud, current_pose);

            PointCloudMessage pointcloud_map_msg;
            pointcloud_map_msg.cloud = laserMapping.getMap();
            pointcloud_map_msg.timestamp = pointcloud_time;
            pointcloud_map_msg.frame_id = "map";

            while (true) {
                if (pointCloudMapBuf.size() < buffer_size && finalOdometryBuf.size() < buffer_size) {
                    
                    finalOdometryBuf_mutex.lock();
                    pointCloudMapBuf_mutex.lock();

                    finalOdometryBuf.push(laserodom_in_msg);
                    pointCloudMapBuf.push(pointcloud_map_msg);

                    finalOdometryBuf_mutex.unlock();
                    pointCloudMapBuf_mutex.unlock();

                    // std::cout << pointcloud_map_msg.timestamp << ": Publish Map and Odomerty" << std::endl;

                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}
