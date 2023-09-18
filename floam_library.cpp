/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

*/

#include <Parameters.h>
#include <SLAMBenchAPI.h>
#include <io/SLAMFrame.h>
#include <io/sensor/LidarSensor.h>
#include <io/sensor/GroundTruthSensor.h>


#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>

#include <thread>
#include <chrono>
#include <iostream>
#include <string>

#include "include/floam.h"

const std::string default_yaml_path = "/deps/floam/configs/configs.yaml";

// Parameters
std::string yaml_path;
std::string dataset_name;
bool show_point_cloud;

// Sensors
slambench::io::LidarSensor *lidar_sensor;

slambench::TimeStamp last_frame_timestamp;
double current_timestamp;

// Outputs
slambench::outputs::Output *pose_output;
slambench::outputs::Output *pointcloud_output;

// System
FLOAM floam;
std::pair<OdometryMessage, PointCloudMessage> output_pair;

// contains rotation only
Eigen::Matrix4f align_mat = (Eigen::Matrix4f() <<  0.0, -1.0,  0.0, 0.0,
                                                   0.0,  0.0, -1.0, 0.0,
                                                   1.0,  0.0,  0.0, 0.0,
                                                   0.0,  0.0,  0.0, 1.0).finished();


bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings) {

    slam_settings->addParameter(TypedParameter<std::string>("configs", "configuration", "path to configuration YAML file", &yaml_path, &default_yaml_path));

    return true;
}


bool sb_init_slam_system(SLAMBenchLibraryHelper *slam_settings) {
    
    // Declare Outputs
    pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);

    pointcloud_output = new slambench::outputs::Output("PointCloud", slambench::values::VT_POINTCLOUD, true);
    pointcloud_output->SetKeepOnlyMostRecent(true);

    slam_settings->GetOutputManager().RegisterOutput(pose_output);
    slam_settings->GetOutputManager().RegisterOutput(pointcloud_output);

    // Inspect sensors
    lidar_sensor = (slambench::io::LidarSensor*)slam_settings->get_sensors().GetSensor(slambench::io::LidarSensor::kLidarType);
    if (lidar_sensor == nullptr) {
        std::cerr << "Invalid sensors found, Lidar not found." << std::endl;
        return false;
    }

    // ================================Read YAML================================
    YAML::Node config = YAML::LoadFile(yaml_path);

    scan_line = config["scan_line"].as<int>();
    vertical_angle = config["vertical_angle"].as<double>();
    scan_period = config["scan_period"].as<double>();
    max_dis = config["max_dis"].as<double>();
    min_dis = config["min_dis"].as<double>();
    map_resolution = config["map_resolution"].as<double>();
    buffer_size = config["buffer_size"].as<int>();
    dataset_name = config["dataset_name"].as<std::string>();
    show_point_cloud = config["show_point_cloud"].as<bool>();

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
    std::cout << "dataset_name: " << dataset_name << std::endl;
    std::cout << "show_point_cloud: " << show_point_cloud << std::endl;

    // ================================Start Fast-LOAM================================
    if (!floam.Init()) {
        std::cerr << "Failed to initialize slam system." << std::endl;
        return false;
    }
    std::cout << "Fast-LOAM initialized" << std::endl;

    return true;
}


bool sb_update_frame(SLAMBenchLibraryHelper *slam_settings , slambench::io::SLAMFrame *s) {
    
	if (s->FrameSensor == lidar_sensor) {

        PointCloudMessage pointCloudInMsg;
        pointCloudInMsg.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

        last_frame_timestamp = s->Timestamp;
        current_timestamp = static_cast<double>(s->Timestamp.S) + static_cast<double>(s->Timestamp.Ns) / 1e9;
        pointCloudInMsg.timestamp = current_timestamp;

        void* rawData = s->GetData();
        size_t dataSize = s->GetVariableSize(); // Assuming you have such a method. If not, you'd need another way to know the size.

        char* byteData = reinterpret_cast<char*>(rawData);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

        for (size_t i = 0; i < dataSize; i += sizeof(pcl::PointXYZI)) {
            pcl::PointXYZI point = *reinterpret_cast<pcl::PointXYZI*>(byteData + i);
            cloud->points.push_back(point);
        }
        pointCloudInMsg.cloud = cloud;

        pointCloudInMsg.cloud->height = 1;
        pointCloudInMsg.cloud->width = cloud->points.size();

        floam.AdjustInput(pointCloudInMsg);

        return true;
	}
	
	return false;
}


bool sb_process_once(SLAMBenchLibraryHelper *slam_settings) {
    
    output_pair = floam.Process();

    return true;
}


bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *ts_p) {
    (void)lib;

    slambench::TimeStamp ts = *ts_p;

    if (pose_output->IsActive()) {
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());

        OdometryMessage output_cdom_msg = output_pair.first;
        Eigen::Quaternionf quat(output_cdom_msg.orientationW, output_cdom_msg.orientationX, output_cdom_msg.orientationY, output_cdom_msg.orientationZ);
        Eigen::Matrix3f rotation = quat.toRotationMatrix();

        // Extract translation vector
        Eigen::Vector3f translation(output_cdom_msg.positionX, output_cdom_msg.positionY, output_cdom_msg.positionZ);

        // Construct 4x4 transformation matrix
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3, 3>(0, 0) = rotation;
        pose.block<3, 1>(0, 3) = translation;

		pose_output->AddPoint(ts, new slambench::values::PoseValue(align_mat * pose));
    }
    
    if (pointcloud_output->IsActive() && show_point_cloud) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_trans(new pcl::PointCloud<pcl::PointXYZI>);
        PointCloudMessage output_cloud_msg = output_pair.second;

        pcl::transformPointCloud(*(output_cloud_msg.cloud), *cloud_out_trans, align_mat);

        auto slambench_point_cloud = new slambench::values::PointCloudValue();
        int count = 0;
        for(const auto &p : *cloud_out_trans) {
            if (count % 1 == 0) slambench_point_cloud->AddPoint(slambench::values::Point3DF(p.x, p.y, p.z));
            count++;
        }

        // Take lock only after generating the map
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        pointcloud_output->AddPoint(ts, slambench_point_cloud);
    }

    return true;
}


bool sb_clean_slam_system() {
    delete pose_output;
    delete pointcloud_output;
    delete lidar_sensor;
    return true;
}
