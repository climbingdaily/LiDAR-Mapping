#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/io/pcd_io.h>
#include <sstream>
#include <pcl/filters/voxel_grid.h>
#include <common.h>

typedef pcl::octree::OctreePointCloudSearch<PointType> OctreePointCloudSearch;
typedef Eigen::Matrix4f MatrixType;
typedef OctreePointCloudSearch::Ptr OctreePtr;

class MapManager{
public:
	MapManager(OctreePointCloudSearch::Ptr &ot, PointCloud::Ptr &mp);
	~MapManager();
	void AddFrameToMap(PointCloud::Ptr frame);
	void UpdateMap(PointCloud::Ptr cur_frame);
private:
	OctreePointCloudSearch::Ptr octree;
	PointCloud::Ptr map;
	pcl::VoxelGrid<PointType> vg;
	double rl;
};
#endif
