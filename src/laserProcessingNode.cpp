// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "laserProcessingNode.h"

void laserProcessingNode::adjustLaserMappingInput(const PointCloudMessage &laserCloud)
{
    pointCloudInBuf_mutex.lock();
    pointCloudInBuf.push(laserCloud);
    pointCloudInBuf_mutex.unlock();
}


void laserProcessingNode::laser_processing()
{
    while (1)
    {
        if (!pointCloudInBuf.empty())
        {
            // read data
            pointCloudInBuf_mutex.lock();
            pointcloud_in_msg = pointCloudInBuf.front();
            pointCloudInBuf.pop();
            pointCloudInBuf_mutex.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();

            laserProcessing.featureExtraction(pointcloud_in_msg.cloud, pointcloud_edge, pointcloud_surf);

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;

            // Publish pointcloud_filtered_msg
            pointcloud_filtered_msg.timestamp = pointcloud_in_msg.timestamp;
            pointcloud_filtered_msg.frame_id = "base_link";
            pointcloud_filtered_msg.cloud = pointcloud_filtered;
            

            // Publish pointcloud_edge_msg
            pointcloud_edge_msg.timestamp = pointcloud_in_msg.timestamp;
            pointcloud_edge_msg.frame_id = "base_link";
            pointcloud_edge_msg.cloud = pointcloud_edge;
            

            // Publish pointcloud_surf_msg
            pointcloud_surf_msg.timestamp = pointcloud_in_msg.timestamp;
            pointcloud_surf_msg.frame_id = "base_link";
            pointcloud_surf_msg.cloud = pointcloud_surf;
            

            while (true) {
                if (pointCloudFilteredBuf.size() < buffer_size && pointCloudEdgeBuf.size() < buffer_size && pointCloudSurfBuf.size() < buffer_size) {

                    pointCloudFilteredBuf_mutex.lock();
                    pointCloudFilteredBuf.push(pointcloud_filtered_msg);
                    pointCloudFilteredBuf_mutex.unlock();

                    pointCloudEdgeBuf_mutex.lock();
                    pointCloudEdgeBuf.push(pointcloud_edge_msg);
                    pointCloudEdgeBuf_mutex.unlock();

                    pointCloudSurfBuf_mutex.lock();
                    pointCloudSurfBuf.push(pointcloud_surf_msg);
                    pointCloudSurfBuf_mutex.unlock();

                    std::cout << pointcloud_in_msg.timestamp << ": Publish Filtered, Edge, and Surf cloud" << std::endl;

                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

/*
double total_time =0;
int total_frame=0;

void laser_processing(){
    while(1){
        if(!pointCloudBuf.empty()){
            //read data
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudBuf.pop();
            mutex_lock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            laserProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time+=time_temp;
            //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "base_link";
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "base_link";
            pubEdgePoints.publish(edgePointsMsg);


            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "base_link";
            pubSurfPoints.publish(surfPointsMsg);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessing.init(lidar_param);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyneHandler);

    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100);

    pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100);

    pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100); 

    std::thread laser_processing_process{laser_processing};

    ros::spin();

    return 0;
} */

