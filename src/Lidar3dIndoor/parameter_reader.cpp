#include "parameter_reader.h"

ParameterReader::ParameterReader()
{
    readData();
}

ParameterReader::ParameterReader(string _filename)
{
    filename = _filename;
    readData();
}

ParameterReader::~ParameterReader()
{

}

void ParameterReader::SetParameterFilePath(string _para_file_path)
{
    filename = _para_file_path;
}

void ParameterReader::readData()
{
    ifstream fin( filename.c_str() );
    if (!fin){
        cout << "Parameter file does not exist. Please use SetParameterFilePath(_file_path) to assign correct path." << endl;
        return;
    }
    while(!fin.eof()){
	string str;
	getline( fin, str );
	if (str[0] == '#'){
	    // 以‘＃’开头的是注释
	    continue;
	}

	int pos = str.find("=");
	if (pos == -1)
	    continue;
	string key = str.substr( 0, pos );
	string value = str.substr( pos+1, str.length() );
	data[key] = value;

	if ( !fin.good() )
	    break;
    }
    fin.close();
}

string ParameterReader::getData(string key)
{
    map<string, string>::iterator iter = data.find(key);
    if (iter == data.end())
    {
	cerr<<"Parameter name "<<key<<" not found!"<<endl;
	return string("NOT_FOUND");
    }
    return iter->second;
}
