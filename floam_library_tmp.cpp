/*
#include "include/floam.h"

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

const std::string default_yaml_path = "/home/yuhao/floam/configs/configs.yaml";

int main() {

    YAML::Node config = YAML::LoadFile(default_yaml_path);

    scan_line = config["scan_line"].as<int>();
    vertical_angle = config["vertical_angle"].as<double>();
    scan_period = config["scan_period"].as<double>();
    max_dis = config["max_dis"].as<double>();
    min_dis = config["min_dis"].as<double>();
    map_resolution = config["map_resolution"].as<double>();
    buffer_size = config["buffer_size"].as<int>();

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);
    
    std::cout << "scan_line: " << scan_line << std::endl;
    std::cout << "vertical_angle: " << vertical_angle << std::endl;
    std::cout << "scan_period: " << scan_period << std::endl;
    std::cout << "max_dis: " << max_dis << std::endl;
    std::cout << "min_dis: " << min_dis << std::endl;
    std::cout << "map_resolution: " << map_resolution << std::endl;
    std::cout << "buffer_size: " << buffer_size << std::endl;

    FLOAM floam;

    floam.Init();

    pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);

    std::string darpa_path = "/mnt/d/Download/Dataset/Cerberus/anymal1_sync/velodyne_pcd";
    std::string newer_path = "/mnt/d/Download/Dataset/Newer/short_01_sync/ouster_scan";
    std::string kitti_path = "/mnt/d/Download/Dataset/KITTI/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.dir/velodyne_points/data";
    boost::filesystem::path directory_path("/mnt/d/Download/Dataset/Cerberus/anymal1_sync/velodyne_pcd"); // replace with your directory path

    int count = 0;
    for (boost::filesystem::directory_iterator it(directory_path); it != boost::filesystem::directory_iterator(); ++it) {
        
        count++;

        if (count > 500) break;

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
            floam.AdjustInput(pointCloudInMsg);

            auto output_pair = floam.Process();
            OdometryMessage pose_msg = output_pair.first;
            PointCloudMessage cloud_msg = output_pair.second;

            if (pose_msg.timestamp == 0.0) {
                break;
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

    return 0;
}
*/