#include "gicp_end.h"

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <pcl/registration/ia_ransac.h>

GicpEnd::GicpEnd()
{
}

GicpEnd::~GicpEnd()
{
}

#if 0

void init(){
    Global = Matrix4Type::Identity ();
    usePCL = para.loadPara<bool>("usePCL");
    globalFirst = para.loadPara<bool>("globalFirst");
    showCloud = para.loadPara<bool>("showCloud_GICP");
    reader.setPcapPath(para.getData("fileNamePcap"));
    reader.setCalibrationPath(para.getData("calibrationPath"));
    reader.OpenPcap();
}

void GicpEnd::setOutputFile(string _out_file)
{
    vertex_file.open( _out_file.c_str() );
    if(!vertex_file){
      std::cout << "cannot create file " << _out_file << endl;
      exit(-1);
    }
}


int GicpEnd::Run()
{    
    OctreePointCloudSearch::Ptr octree(new OctreePointCloudSearch(para.loadPara<double>("octree_resolution")));
    PointCloud::Ptr cur(new PointCloud), cur_1(new PointCloud), pre(new PointCloud), map(new PointCloud);
    Matrix4Type Tf, Tm;
    
    //-------------------------------------------------------
    //          Read first frame and build octree
    //          Skip the first frame if too sparse
    reader.LoadFirstFrame(pre);
    saveTrack();
    if(!pre) return 1;
    *map = *pre;
    MapManager mg(octree, map);
    //-------------------------------------------------------

    while (1){
        if(reader.readPointCloud(cur) == -1)
	    break;
	reader.VoxelGrid(cur);
	
        if(globalFirst){ 
		pcl::transformPointCloud (*cur, *cur_1, Global);
			    
		if(usePCL)  PairAlign_PCL (pre, cur_1, Tf);
		else  PairAlign_Segal (pre, cur_1, Tf);
			
		Global = Tf * Global; 
		pcl::transformPointCloud (*cur, *cur, Global); // cur to pre
#if 0	
		AlignFrameWithMap(octree, cur_1, Tm);
		Global = Tm * Global;
		pcl::transformPointCloud (*cur, *cur, Global);
#endif
		*pre = *cur;
	}
	else{	
	   	if(usePCL)    PairAlign_PCL (pre, cur, Tf);
		else    PairAlign_Segal (pre, cur, Tf);
		*pre = *cur;
		Global = Global * Tf; 
		pcl::transformPointCloud (*cur, *cur, Global);	
			
		AlignFrameWithMap(octree, cur, Tm); 
		pcl::transformPointCloud (*cur, *cur, Tm);
		Global = Tm * Global;
        }
        
        mg.AddFrameToMap(cur);  
        if(!vertex_file){

	}
	saveTrack();

        if(subMapFull(reader))  
	    mg.UpdateMap(cur);
	if(showCloud)  
	    debug.ShowCloud(map); 
    }
    vertex_file.close();
    return 0;
}

void GicpEnd::SetShowMap(bool _shcl_flag)
{
    showCloud = _shcl_flag;
    if(showCloud)
      debug.p = new pcl::visualization::PCLVisualizer("Cloud in GICP");
}


double GicpEnd::PairAlign_Segal(const pcl::PointCloud< PointType >::Ptr base, const pcl::PointCloud< PointType >::Ptr data, Eigen::Matrix4f& pairTransform)
{
    dgc::gicp::GICPPointSet scan1, scan2;
    for(int i=0; i<base->points.size(); ++i){
      dgc::gicp::GICPPoint p;
      p.x = base->points[i].x;
      p.y = base->points[i].y;
      p.z = base->points[i].z;
      scan1.AppendPoint(p);
    }
    for(int i=0; i<data->points.size(); ++i){
      dgc::gicp::GICPPoint p;
      p.x = data->points[i].x;
      p.y = data->points[i].y;
      p.z = data->points[i].z;
      scan2.AppendPoint(p);
    }

    dgc_transform_t t_final, t_base, t0, t1;
  
    // set up the transformations
    dgc_transform_identity(t_base);
    dgc_transform_identity(t0);
    dgc_transform_identity(t1);
    dgc_transform_copy(t_final, t_base);
  
    // build kdtrees and normal matrices
    scan1.SetGICPEpsilon(para.loadPara<float>("gicp_epsilon"));
    scan1.BuildKDTree();
    scan1.ComputeMatrices();
    //scan1.SetMaxIteration(para.loadPara<int>("MaximumIteration"));
    //scan1.SetMaxIterationInner(para.loadPara<int>("MaximumIterationInner"));
  
    scan2.SetGICPEpsilon(para.loadPara<float>("gicp_epsilon")); 
    scan2.BuildKDTree();
    scan2.ComputeMatrices();

    // align
    double delta = scan1.AlignScan(&scan2, t_base, t1, para.loadPara<float>("MaxCorrespondenceDistance"));
    dgc_transform_left_multiply(t_final, t1); 
    
    // get transform
    for(int i=0; i<4; ++i)
            for(int j=0; j<4; ++j)
                pairTransform(i,j) = (float) t_final[i][j];
	    
    return delta;
}
#endif
/* 
* 
*/
void voxelFilter(PointCloud::Ptr &cloud_in, PointCloud::Ptr &cloud_out, float gridsize)
{
   pcl::VoxelGrid<PointType> vox_grid;
   vox_grid.setLeafSize(gridsize, gridsize, gridsize);
   vox_grid.setInputCloud(cloud_in);
   vox_grid.filter(*cloud_out);
   return;
}

pcl::PointCloud<pcl::Normal>::Ptr getNormals(PointCloud::Ptr cloud, double radius)
{
   pcl::PointCloud<pcl::Normal>::Ptr normalsPtr(new pcl::PointCloud<pcl::Normal>);
   pcl::NormalEstimation<PointType, pcl::Normal> norm_est;
   norm_est.setInputCloud(cloud);
   norm_est.setRadiusSearch(radius);
   norm_est.compute(*normalsPtr);
   return normalsPtr;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFeatures(PointCloud::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius)
{
   pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
   pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
   pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
   fpfh_est.setInputCloud(cloud);
   fpfh_est.setInputNormals(normals);
   fpfh_est.setSearchMethod(tree);
   fpfh_est.setRadiusSearch(radius);
   fpfh_est.compute(*features);
   return features;
}

Eigen::Matrix4f sac_ia_align(PointCloud::Ptr source, PointCloud::Ptr target,
                             pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_feature, pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_feature,
                             int max_sacia_iterations, double min_correspondence_dist, double max_correspondence_dist)
{
   pcl::SampleConsensusInitialAlignment<PointType, PointType, pcl::FPFHSignature33> sac_ia;
   Eigen::Matrix4f final_transformation;
   sac_ia.setInputSource(source);
   sac_ia.setSourceFeatures(source_feature);
   sac_ia.setInputTarget(target);
   sac_ia.setTargetFeatures(target_feature);
   sac_ia.setMaximumIterations(max_sacia_iterations);
   sac_ia.setMinSampleDistance(min_correspondence_dist);
   sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);
   PointCloud::Ptr finalcloud(new PointCloud);
   sac_ia.align(*finalcloud);
   final_transformation = sac_ia.getFinalTransformation();
   return final_transformation;
}

double GicpEnd::PairAlign_PCL(const PointCloud::Ptr base, const PointCloud::Ptr data, Matrix4Type &pairTransform)
{
   // // float VOXEL_GRID_SIZE = 0.01;
   // double radius_normal = 0.2;
   // double radius_feature = 0.3;
   // double max_sacis_iteration = 40;
   // double min_correspondence_dist = 0.03;
   // double max_correspondence_dist = 7;
   // PointCloud::Ptr result(new PointCloud);
   // //计算法向量
   // pcl::PointCloud<pcl::Normal>::Ptr source_normal(new pcl::PointCloud<pcl::Normal>);
   // pcl::PointCloud<pcl::Normal>::Ptr target_normal(new pcl::PointCloud<pcl::Normal>);
   // source_normal = getNormals(data, radius_normal);
   // target_normal = getNormals(base, radius_normal);

   // //计算FPFH特征
   // pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_feature(new pcl::PointCloud<pcl::FPFHSignature33>);
   // pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_feature(new pcl::PointCloud<pcl::FPFHSignature33>);
   // source_feature = getFeatures(data, source_normal, radius_feature);
   // target_feature = getFeatures(base, target_normal, radius_feature);

   // //SAC-IA配准
   // Eigen::Matrix4f init_transform;
   // init_transform = sac_ia_align(data, base, source_feature, target_feature, max_sacis_iteration, min_correspondence_dist, max_correspondence_dist);
   // cout << init_transform << endl;
   
   // pcl::transformPointCloud(*data, *result, init_transform);

   PointCloud::Ptr transformed(new PointCloud);
   pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
   gicp.setInputSource(data);
   gicp.setInputTarget(base);
   gicp.setRotationEpsilon(1e-9);
   gicp.setTransformationEpsilon(1e-8);
   gicp.setMaxCorrespondenceDistance(4.5);
   gicp.setMaximumOptimizerIterations(40);
   gicp.align(*transformed);
   pairTransform = gicp.getFinalTransformation();

   // cout << gicp.getFinalTransformation() << endl;
   float score = gicp.getFitnessScore();

   // PointCloud::Ptr transformed2(new PointCloud);
   // pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp2;
   // gicp2.setInputTarget(base);
   // gicp2.setInputSource(transformed);
   // gicp2.setRotationEpsilon(1e-9);
   // gicp2.setTransformationEpsilon(1e-8);
   // gicp2.setMaxCorrespondenceDistance(1);
   // gicp2.setMaximumOptimizerIterations(30);
   // gicp2.setEuclideanFitnessEpsilon(0.01);
   // gicp2.align(*transformed2);
   // pairTransform = gicp2.getFinalTransformation() * pairTransform;
   // cout << gicp2.getFinalTransformation() << endl;
   // cout << gicp2.getFitnessScore() << endl;
   return score;
}

double GicpEnd::UseNeighborsAlign(pcl::octree::OctreePointCloudSearch<PointType>::Ptr octree, PointCloud::Ptr cur_frame, MatrixType &pairTransform)
{
   PointCloud::Ptr neighbors(new PointCloud);
   PointType searchPoint;

   // use K nearest method find neighbors
   int K = 3;
   std::vector<int> pointIdxNKNSearch;
   std::vector<float> pointNKNSquaredDistance;
   for (size_t i = 0; i < cur_frame->size(); ++i)
   { // find neighbor
      searchPoint = cur_frame->points[i];
      if (octree->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
         for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            neighbors->points.push_back(octree->getInputCloud()->points[pointIdxNKNSearch[i]]);
      }
   }
   voxelFilter(neighbors, neighbors, 0.05);
   neighbors->height = 1;
   neighbors->width = neighbors->points.size();
   neighbors->is_dense = true;
   return PairAlign_PCL(neighbors, cur_frame, pairTransform);
}

#if 0
void GicpEnd::AlignFrameWithMap(pcl::octree::OctreePointCloudSearch< PointType >::Ptr octree, pcl::PointCloud< PointType >::Ptr cur_frame, MatrixType& pairTransform)
{
    UseNeighborsAlign(octree, cur_frame, pairTransform);
}

void GicpEnd::SetSubMapSize(int _smps)
{
    subMapSize = _smps;
}


bool GicpEnd::subMapFull(PointCloudReader& reader)
{
    return !(reader.frameNumber % para.loadPara<int>("subMapSize"));
}

void GicpEnd::saveTrack()
{
    vertex_file << g2o_tool.transform2Vertex(reader.frameNumber, Global) << endl;
}
#endif