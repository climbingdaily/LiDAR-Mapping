#pragma once
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "common.h"
#include "pcapFileReader.h"
#include "point_cloud_reader.h"
#include "tool_func.h"

using namespace std;

class TimeStampReader
{
public:
   void setPcapFile(string _pcap_file);
   void setCalibFile(string _calib_file);
   double getTimpStamp(long long frameId);
   int openPcap(string _pcap_file, string _calib_file);

private:
   long long frameNumber;
   pcapReader reader;
   veloFrame frame;
   string pcap_file;
   string calib_file;
};

class AddTimeStamp
{
public:
    void setPcapFile(string _pcap_file);
    void setCalibFile(string _calib_file);
    void setInputTrack(string _input);
    void setOutputTrack(string _output);
    void setOutputFolder(string _opf){outputfolder = _opf;};
    void Run();
private:
    string formatg2oString(string& _input);
private:
    ifstream track_input;
    ofstream track_output;
    TimeStampReader ts_reader;
    string pcap_file;
    string calib_file;
    string outputfolder;
    G2oToolFunc g2o;
};