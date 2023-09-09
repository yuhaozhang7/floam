#include "laserProcessingNode.h"
#include "odomEstimationNode.h"
#include "laserMappingNode.h"
#include "utility.h"

#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

int main() {

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessingNode LP;
    LP.laserProcessing.init(lidar_param);
    std::thread laser_processing_thread(&laserProcessingNode::laser_processing, &LP);

    odomEstimationNode OE;
    OE.odomEstimation.init(lidar_param, map_resolution);
    std::thread odom_estimation_thread(&odomEstimationNode::odom_estimation, &OE);

    laserMappingNode LM;
    LM.laserMapping.init(map_resolution);
    std::thread laser_mapping_thread(&laserMappingNode::laser_mapping, &LM);

    pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);

    std::string darpa_path = "/mnt/d/Download/Dataset/Cerberus/anymal1_sync/velodyne_pcd";
    std::string newer_path = "/mnt/d/Download/Dataset/Newer/short_01_sync/ouster_scan";
    std::string kitti_path = "/mnt/d/Download/Dataset/KITTI/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.dir/velodyne_points/data";
    boost::filesystem::path directory_path("/mnt/d/Download/Dataset/Newer/short_01_sync/ouster_scan"); // replace with your directory path

    int count = 0;
    for (boost::filesystem::directory_iterator it(directory_path); it != boost::filesystem::directory_iterator(); ++it) {
        
        count++;

        if (count > 3000) break;

        if (boost::filesystem::is_regular_file(it->path()) && it->path().extension() == ".pcd") {\

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudIn(new pcl::PointCloud<pcl::PointXYZI>());

            if (pcl::io::loadPCDFile<pcl::PointXYZI>(it->path().string(), *pointCloudIn) == -1) {
                std::cerr << "Couldn't read file " << it->path().string() << std::endl;
                continue;
            }

            PointCloudMessage pointCloudInMsg;
            pointCloudInMsg.cloud = pointCloudIn;
            pointCloudInMsg.timestamp = count;

            // Wait if the buffer size is greater than or equal to 5
            while (true) {
                if (pointCloudInBuf.size() < buffer_size) {
                    pointCloudInBuf_mutex.lock();
                    pointCloudInBuf.push(pointCloudInMsg);
                    pointCloudInBuf_mutex.unlock();
                    break; // break out of the while loop
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep for a short duration before checking again
            }

            auto output_pair = getMap();
            OdometryMessage pose_msg = output_pair.first;
            PointCloudMessage cloud_msg = output_pair.second;

            if (pose_msg.timestamp == 0.0) {
                std::cout << "Timestamp is zero!!!!!" << std::endl;
                stopFlag.store(true);
            } else {
                pcl::PointXYZ point;
                point.x = pose_msg.positionX;
                point.y = pose_msg.positionY;
                point.z = pose_msg.positionZ;
                trajectory->points.push_back(point);

                map = cloud_msg.cloud;
                std::cout << pose_msg.timestamp << " " << cloud_msg.timestamp << ": Get the Output" << std::endl;
            }
        }
    }
    stopFlag.store(true);
    /*
    while (true) {
        if (laserOdometryBuf.size() == 0) break;

        laserOdometryBuf_mutex.lock();
        OdometryMessage pose = laserOdometryBuf.front();
        laserOdometryBuf.pop();
        laserOdometryBuf_mutex.unlock();

        pcl::PointXYZ point;
        point.x = pose.positionX;
        point.y = pose.positionY;
        point.z = pose.positionZ;

        trajectory->points.push_back(point);
    } */

    std::string filename = "/mnt/d/Download/Dataset/Cerberus/trajectory_test.pcd";
    trajectory->width = trajectory->points.size();
    trajectory->height = 1;
    if (pcl::io::savePCDFile<pcl::PointXYZ>(filename, *trajectory) == -1) {
        PCL_ERROR("Couldn't write to PCD file \n");
        return -1;
    }
    std::cout << "Saved " << trajectory->points.size() << " data points to " << filename << "." << std::endl;

    
    filename = "/mnt/d/Download/Dataset/Cerberus/map_test.pcd";
    if (pcl::io::savePCDFile<pcl::PointXYZI>(filename, *map) == -1) {
        PCL_ERROR("Couldn't write to PCD file \n");
        return -1;
    }
    std::cout << "Saved " << map->points.size() << " data points to " << filename << "." << std::endl;

    laser_processing_thread.join();
    odom_estimation_thread.join();
    laser_mapping_thread.join();

    return 0;
}
