#include "map_manager.h"

MapManager::MapManager(OctreePointCloudSearch::Ptr &ot, PointCloud::Ptr &mp)
{
    map = mp;
    octree = ot;
    rl = octree->getResolution();
    octree->setInputCloud(map);
    octree->addPointsFromInputCloud();
}

MapManager::~MapManager() {}

void MapManager::AddFrameToMap(PointCloud::Ptr frame)
{
    PointType tempPoint;
    for (int i = 0; i < frame->points.size(); i++)
    { // Add frame to Global Map
        tempPoint.x = frame->points[i].x;
        tempPoint.y = frame->points[i].y;
        tempPoint.z = frame->points[i].z;
        tempPoint.intensity = frame->points[i].intensity;
        tempPoint.data_n[0] = frame->points[i].data_n[0];
        tempPoint.data_n[1] = frame->points[i].data_n[1];

        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
        bool isInBox = (tempPoint.x >= min_x && tempPoint.x <= max_x) && (tempPoint.y >= min_y && tempPoint.y <= max_y) && (tempPoint.z >= min_z && tempPoint.z <= max_z);
        if (!isInBox || !octree->isVoxelOccupiedAtPoint(tempPoint))
            // if (!octree->isVoxelOccupiedAtPoint(tempPoint))
            octree->addPointToCloud(tempPoint, map);
    }
}

void MapManager::UpdateMap(PointCloud::Ptr cur_frame)
{
    octree->deleteTree();
    *map = *cur_frame;
    PointCloud::Ptr map_filtered(new PointCloud);
    vg.setInputCloud(map);
    vg.setLeafSize(rl, rl, rl);
    vg.filter(*map_filtered);

    *map = *map_filtered;
    octree->setInputCloud(map);
    octree->addPointsFromInputCloud();
}
