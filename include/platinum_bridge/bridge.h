#ifndef BRIDGE
#define BRIDGE

#include "ros/ros.h"
#include <ros/package.h>
#include <string>
#include <vector>
#include <sstream>
#include <tinyxml.h>
#include <map>

using namespace std;

// Ranges of the paramters in levels
enum Level {LV1, LV2, LV3};

typedef map<string, vector<float>> NamedList;
typedef map<string, Level> NamedLevel;
typedef map<string, NamedLevel> LevelMap;

// Data strtcure to include all cohan params to tune
struct CoHANParams{
	NamedList RParam;
	NamedList HParam;
	NamedList SNorm;
};

// Data structure to define the params in terms of levels
struct ParamList{
	string type;
	vector<Level> params;
};

// Data structure for defining context
struct Context{
	string task;
	string human_type;
};

// class defining the CoHAN and Platinum Bridge
class PlatinumToCohan{
public:
	PlatinumToCohan();

	~PlatinumToCohan();

	// Updates the parameters on server based on the Context given
	bool setParams(Context context);

	// Returns the existing params formatted as ParamList
	ParamList getParams(string type="all");

private:
	// Reads the params and contexts from the xml file
	bool readParamXML();
	bool readContextXML();
	vector<Level> toLevels(string s, string delimiter);
	Level toLevel(string s);
	vector<float> toFloats(string s, string delimiter); 

	// Stores the params from the xml
	CoHANParams cohan_params_;

	//Stores the contexts from the xml
	LevelMap contexts_;
	LevelMap tasks_;
	LevelMap humans_;

	string map_name_;
	TiXmlDocument* param_doc_;
	TiXmlDocument* context_doc_;

	//Params list
	map<string, string> robot_param_names_, human_param_names_, social_param_names_;
								
};

#endif
