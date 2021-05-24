#include "submap.h"
#include "common.h"
/*********************************************************
 * class SubMap:
 *
 * ******************************************************/
void SubMap::SaveTraj(std::string _path, long long _traj_id)
{
    g2o.makeDir(_path);
    std::string outputfilePath = _path +"/"+ "traj_center_" + g2o.toString(_traj_id)+".txt";
    std::ofstream traj_output(outputfilePath.c_str());
    Vector3Type temp_pos_;
    for(Trajectory::iterator traj_it=traj.begin(); traj_it != traj.end(); ++traj_it){
      traj_output << g2o.transform2Vertex(traj_it->frame_id, traj_it->transform) << endl;
      //temp_pos_= g2o.getTranslation(traj_it->transform);
      //traj_output << temp_pos_(0) << " " << temp_pos_(1) << " " << temp_pos_(2) << endl;
    }
}

double SubMap::Distance(SubMap s)
{
    Matrix4Type t1 = s.center.transform;
    Matrix4Type t2 = center.transform;
    return g2o.getDistance(t1, t2);
}

void SubMap::GetCenter()
{
    int n = traj.size();
    if(n == 0)
    {
        std::cout << "The trajectory of this submap is not exist!." << std::endl;
	return;
    }
    center = traj[n/2];
}

/*********************************************************
 * class SubMap2:
 *
 * ******************************************************/
void SubMap2::SaveTraj(string _path)
{
    g2o.makeDir(_path);
    long long int _traj_id = GetCenterPosId();
    std::string outputfilePath = _path +"/"+ "traj_center_" + g2o.toString(_traj_id)+".txt";
    std::ofstream traj_output(outputfilePath.c_str());
    Vector3Type temp_pos_;
    for(PosSet::iterator traj_it=traj.begin(); traj_it != traj.end(); ++traj_it){
      traj_output << g2o.transform2Vertex(traj_it->frame_id, traj_it->transform) << endl;
      //temp_pos_= g2o.getTranslation(traj_it->transform);
      //traj_output << temp_pos_(0) << " " << temp_pos_(1) << " " << temp_pos_(2) << endl;
    }
}

void SubMap2::ShowTraj(DebugToolFunc *debug_tool)
{
    PointCloud::Ptr cloud(new PointCloud);
    for(PosSet::iterator it=traj.begin(); it != traj.end(); ++it)
    {
      Matrix4Type tmp_trans = it->transform;
      Vector3Type tmp_t =  g2o.getTranslation(tmp_trans);
      PointType pt;
      pt.x = tmp_t(0);
      pt.y = tmp_t(1);
      pt.z = tmp_t(2);
      cloud->points.push_back(pt);
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = true;
    debug_tool->ShowCloud(cloud, GetCenterPosId());
}


long long int SubMap2::GetCenterPosId()
{
    Pos b,e;
    b = *(traj.begin());
    e = *(traj.rbegin());
    long long int centerId = (b.frame_id+e.frame_id)/2;

    Pos tmpPos;
    tmpPos.frame_id = centerId;
    tmpPos.transform = Matrix4Type::Identity();

    while(traj.find(tmpPos) == traj.end()) // frame_id in loam result is not continuous by step=1
                                             // so there is maybe not exist centerId=(begin+end)/2
    {
       (tmpPos.frame_id)++;
    }
    return tmpPos.frame_id;
}

