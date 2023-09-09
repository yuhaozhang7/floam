// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "laserProcessingNode.h"

void laserProcessingNode::adjustFLOAMInput(const PointCloudMessage &laserCloud)
{
    pointCloudInBuf_mutex.lock();
    pointCloudInBuf.push(laserCloud);
    pointCloudInBuf_mutex.unlock();
}


void laserProcessingNode::laser_processing()
{
    while (!stopFlag.load())
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

            laserProcessing.featureExtraction(pointcloud_in_msg.cloud, pointcloud_edge, pointcloud_surf);

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;

            // Publish pointcloud_filtered_msg
            PointCloudMessage pointcloud_filtered_msg;
            pointcloud_filtered_msg.timestamp = pointcloud_in_msg.timestamp;
            pointcloud_filtered_msg.frame_id = "base_link";
            pointcloud_filtered_msg.cloud = pointcloud_filtered;
            

            // Publish pointcloud_edge_msg
            PointCloudMessage pointcloud_edge_msg;
            pointcloud_edge_msg.timestamp = pointcloud_in_msg.timestamp;
            pointcloud_edge_msg.frame_id = "base_link";
            pointcloud_edge_msg.cloud = pointcloud_edge;
            

            // Publish pointcloud_surf_msg
            PointCloudMessage pointcloud_surf_msg;
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

                    // std::cout << pointcloud_in_msg.timestamp << ": Publish Filtered, Edge, and Surf cloud" << std::endl;

                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}
