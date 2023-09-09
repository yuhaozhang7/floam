#pragma once

#include "laserProcessingNode.h"
#include "odomEstimationNode.h"
#include "laserMappingNode.h"
#include "utility.h"

#include <pcl/io/pcd_io.h>

class FLOAM
{
public:
    FLOAM() = default;

    bool Init();

    void AdjustInput(const PointCloudMessage &pointCloudInMsg);

    std::pair<OdometryMessage, PointCloudMessage> Process();

    std::pair<OdometryMessage, PointCloudMessage> getOutput();

    laserProcessingNode LP;
    std::thread laser_processing_thread;

    odomEstimationNode OE;
    std::thread odom_estimation_thread;

    laserMappingNode LM;
    std::thread laser_mapping_thread;

};