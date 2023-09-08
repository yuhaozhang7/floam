#include "laserProcessingNode.h"
#include "utility.h"

#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

int main() {

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessingNode LP;
    LP.laserProcessing.init(lidar_param);

    std::thread laser_processing_thread(&laserProcessingNode::laser_processing, &LP);

    boost::filesystem::path directory_path("/mnt/d/Download/Dataset/Cerberus/anymal1_sync/velodyne_pcd"); // replace with your directory path
    int count = 0;
    for (boost::filesystem::directory_iterator it(directory_path); it != boost::filesystem::directory_iterator(); ++it) {

        if (count > 5) break;
        count++;

        if (boost::filesystem::is_regular_file(it->path()) && it->path().extension() == ".pcd") {\

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudIn(new pcl::PointCloud<pcl::PointXYZI>());

            if (pcl::io::loadPCDFile<pcl::PointXYZI>(it->path().string(), *pointCloudIn) == -1) {
                std::cerr << "Couldn't read file " << it->path().string() << std::endl;
                continue;
            }

            PointCloudMessage pointCloudInMsg;
            pointCloudInMsg.cloud = pointCloudIn;

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
        }
    }

    PointCloudMessage pointCloudEdgeMsg = pointCloudSurfBuf.front();
    pointCloudEdgeBuf.pop();
    std::string filename = "/mnt/d/Download/Dataset/Cerberus/test.pcd";
    if (pcl::io::savePCDFile<pcl::PointXYZI>(filename, *(pointCloudEdgeMsg.cloud)) == -1) {
        PCL_ERROR("Couldn't write to PCD file \n");
        return -1;
    }

    std::cout << "Saved " << pointCloudEdgeMsg.cloud->points.size() << " data points to " << filename << "." << std::endl;

    laser_processing_thread.join();

    return 0;
}
