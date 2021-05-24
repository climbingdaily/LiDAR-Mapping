#include "add_timestamp.h"
#include <utility>
#include <time.h>
#include <iomanip>
/*******************************************
 * class TimeStampReader:
 *
 * *****************************************/
void TimeStampReader::setPcapFile(string _pcap_file)
{
    pcap_file = _pcap_file;
}

void TimeStampReader::setCalibFile(string _calib_file)
{
    calib_file = _calib_file;
}

int TimeStampReader::openPcap(string _pcap_file, string _calib_file)
{
    pcap_file = _pcap_file;
    calib_file = _calib_file;

    if (reader.open(_pcap_file, _calib_file) == -1)
    {
        cout << "TimeStampReader: Error in opening pcap file." << endl;
        return -1;
    }
    frameNumber = -1;
    return 0;
}

double TimeStampReader::getTimpStamp(long long int frameId)
{
    double curTimeStamp;
    double startTime;
    std::vector<pair<double, int>> timestampList;

    if (frameNumber > frameId)
    {
        cout << "frameId to large" << endl;
        openPcap(pcap_file, calib_file);
    }

    while (frameNumber < frameId)
        frameNumber++;

    if (!reader.capture(frame, frameNumber, startTime))
    {
        //std::cout << "TimeStampReader: End of Frame!" << std::endl;
        return -1;
    }

    // count each point's timestamp, select max counted as the frame's timestamp
    for (int n = 0; n < frame.numLine / 2; n++)
    {
        for (int i = 0; i < frame.lines[n].num / 100; i++)
        {
            curTimeStamp = frame.lines[n].pTimestamp[i];
            bool _flag = true;

            // find this timestamp in list
            for (std::vector<pair<double, int>>::iterator it = timestampList.begin(); it != timestampList.end(); ++it)
            {
                if (it->first == curTimeStamp)
                {
                    _flag = false;
                    it->second++;
                }
            }
            // if not exist, add new timstamp
            if (_flag)
            {
                pair<double, int> _tmp_node;
                _tmp_node.first = curTimeStamp;
                _tmp_node.second = 1;
                timestampList.push_back(_tmp_node);
            }
        }
    }

    // find a timestamp has most count
    int _max_conut = 0;
    for (std::vector<pair<double, int>>::iterator it = timestampList.begin(); it != timestampList.end(); ++it)
    {
        if (it->second > _max_conut)
        {
            _max_conut = it->second;
            curTimeStamp = it->first;
        }
    }
    return curTimeStamp;
}

/*******************************************
 * class AddTimeStamp:
 *
 * ******************************************/
void AddTimeStamp::setCalibFile(string _calib_file)
{
    calib_file = _calib_file;
}

void AddTimeStamp::setPcapFile(string _pcap_file)
{
    pcap_file = _pcap_file;
}

void AddTimeStamp::setInputTrack(string _input)
{
    track_input.open(_input.c_str());
    if (!track_input)
    {
        cout << "AddTimeStamp::setInputTrack : File " << _input << "is not exist!" << endl;
    }
}

void AddTimeStamp::setOutputTrack(string _output)
{
    track_output.open(_output.c_str());
    if (!track_output)
    {
        cout << "AddTimeStamp::setOutputTrack : Cannot create file " << _output << endl;
    }
    time_t tt;
    time(&tt);
    tt = tt + 8 * 3600; // transform the time zone
    tm *t = gmtime(&tt);

    char now[36] = {0};
    sprintf(now, "%d/%02d/%02d",
            t->tm_year + 1900,
            t->tm_mon + 1,
            t->tm_mday);
    // track_output << "# Data Name:ZongMu Basement" << endl;
    // track_output << "# Date:" << now << endl;
    // track_output << "# Data Format" << endl;
    // track_output << "# {iSAM_VERTEX, Frame Number, Position[x|y|z], Quaternion[qx|qy|qz|qw], Timestamp(s)}" << endl;
}

string AddTimeStamp::formatg2oString(string &_input)
{
    // string out = "iSAM_VERTEX ";
    string out =  _input.substr(_input.find(' ') + 1);
    return out;
}

void AddTimeStamp::Run()
{
    PointCloudReader pc_reader;
    int length = pcap_file.length();
    std::string sstr = pcap_file.substr(length - 5, length - 1); 
    pc_reader.setPcapPath(pcap_file);
    pc_reader.setCalibrationPath(calib_file);
    if (!pc_reader.OpenPcap())
    {
        printf("Cannot open pcap file!\n");
        return;
    }

    long long frame_id;
    double curTimeStamp;
    Matrix4Type T;
    consoleProgress(97);
    std::string pcdName, savePcd;
    std::string odomfolder, saveOdom;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    PointCloud::Ptr cloudp(new PointCloud);
    while (1)
    {
        frame_id = g2o.vertex2Transform(track_input, T);
        if (track_input.eof())
            break;

        string outStr = g2o.transform2Vertex(frame_id, T);
        float theta = M_PI / 2.0;
        Eigen::Matrix4f Tx = Eigen::Matrix4f::Identity(); //绕X轴转90°
        Tx(1, 1) = cos(theta);
        Tx(1, 2) = -sin(theta);
        Tx(2, 1) = sin(theta);
        Tx(2, 2) = cos(theta);

        Eigen::Matrix4f Ty = Eigen::Matrix4f::Identity(); //绕X轴转90°
        Ty(0, 0) = cos(theta);
        Ty(0, 2) = sin(theta);
        Ty(2, 0) = -sin(theta);
        Ty(2, 2) = cos(theta);

        Eigen::Matrix4f Tz = Eigen::Matrix4f::Identity(); //绕z轴转90°
        Tz(0, 0) = cos(theta);
        Tz(0, 1) = -sin(theta);
        Tz(1, 0) = sin(theta);
        Tz(1, 1) = cos(theta);

        Eigen::Matrix4f T3 = Tz * Tx;

        if (sstr.compare(".pcap") == 0)
        {
            pc_reader.frameNumber = frame_id;
            if (pc_reader.readPointCloud(cloudp) == -1)
                continue;
            if (cloudp->points.size() < 1000)
                continue;
            // curTimeStamp = cloudp->points[0].data_n[0] + cloudp->points[0].data_n[1];
            // curTimeStamp = ts_reader.getTimpStamp(frame_id);
            curTimeStamp = pc_reader.startTimeStamp;
            stringstream ss;
            ss << std::fixed << std::setprecision(9) << curTimeStamp;
            pcdName = ss.str();
            replace(pcdName.begin(), pcdName.end(), '.', '_');
            odomfolder = outputfolder + '/' + pcdName;
            pcl::copyPointCloud(*cloudp, *cloud);
            // pcl::transformPointCloud(*cloud, *cloud, Tx * Tx);
        }
        else
        {
            curTimeStamp = pc_reader.getTimpStamp(frame_id); //直接从pcd的文件名读出时间戳
            pcdName = pc_reader.getPcdById(frame_id);
            getCloudFromPcd(cloud, pcap_file + '/' + pcdName);
            replace(pcdName.begin(), pcdName.end(), '.', '_');
            odomfolder = outputfolder + '/' + pcdName.substr(0,pcdName.rfind('_'));
        }
        savePcd = odomfolder + ".pcd";
        pcl::io::savePCDFileBinary(savePcd, *cloud);
        saveOdom = odomfolder + ".odom";
        std::ofstream ofs(saveOdom);
        ofs << T;
        ofs.close();

        // curTimeStamp *= 1e6; //保存微秒
        track_output << formatg2oString(outStr) << " " << std::fixed << std::setprecision(6) << (curTimeStamp)<< endl;
    }
    consoleProgress(100);
    track_input.close();
    track_output.close();
}
