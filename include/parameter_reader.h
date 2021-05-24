#ifndef PARAMETER_READER_H
#define PARAMETER_READER_H
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
using namespace std;

class ParameterReader
{
  public:
    ParameterReader();
    ParameterReader(string _filename);
    ~ParameterReader();
    void SetParameterFilePath(string _para_file_path);
    string getData(string key);
  private:
    void readData();
    map<string, string> data;
    std::string filename;
};
#endif
