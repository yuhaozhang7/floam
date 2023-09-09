// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationNode.h"

bool is_odom_inited = false;

void odomEstimationNode::odom_estimation(){
    while(!stopFlag.load()){
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            //read data
            pointCloudEdgeBuf_mutex.lock();
            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front().timestamp < pointCloudSurfBuf.front().timestamp-0.5*lidar_param.scan_period)){
                pointCloudEdgeBuf.pop();
                std::cout << "time stamp unaligned with extra point cloud, pls check your data --> odom correction" << std::endl;
                pointCloudEdgeBuf_mutex.unlock();
                continue;
            }

            pointCloudSurfBuf_mutex.lock();
            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front().timestamp < pointCloudEdgeBuf.front().timestamp-0.5*lidar_param.scan_period)){
                pointCloudSurfBuf.pop();
                std::cout << "time stamp unaligned with extra point cloud, pls check your data --> odom correction" << std::endl;
                pointCloudSurfBuf_mutex.unlock();
                continue;  
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());

            pointcloud_edge_in = pointCloudEdgeBuf.front().cloud;
            pointcloud_surf_in = pointCloudSurfBuf.front().cloud;
            double pointcloud_time = pointCloudSurfBuf.front().timestamp;

            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            // pointCloudFilteredBuf_mutex.lock(); // ----------------------------------
            // pointCloudFilteredBuf.pop(); // ----------------------------------
            // pointCloudFilteredBuf_mutex.unlock(); // ----------------------------------

            pointCloudEdgeBuf_mutex.unlock();
            pointCloudSurfBuf_mutex.unlock();

            if (is_odom_inited == false) {
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                std::cout <<"odom inited" << std::endl;
            } else {
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
            }

            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            // static tf::TransformBroadcaster br;
            // tf::Transform transform;
            // transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            // tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            // transform.setRotation(q);
            // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            // publish odometry
            OdometryMessage laser_odometry_msg;
            laser_odometry_msg.frame_id = "map";
            laser_odometry_msg.child_frame_id = "base_link";
            laser_odometry_msg.timestamp = pointcloud_time;
            laser_odometry_msg.orientationX = q_current.x();
            laser_odometry_msg.orientationY = q_current.y();
            laser_odometry_msg.orientationZ = q_current.z();
            laser_odometry_msg.orientationW = q_current.w();
            laser_odometry_msg.positionX = t_current.x();
            laser_odometry_msg.positionY = t_current.y();
            laser_odometry_msg.positionZ = t_current.z();
            
            laserOdometryBuf_mutex.lock();
            laserOdometryBuf.push(laser_odometry_msg);
            laserOdometryBuf_mutex.unlock();

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}
