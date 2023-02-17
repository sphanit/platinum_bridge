#ifndef BRIDGE
#define BRIDGE

#include "ros/ros.h"
#include <string>
#include <vector>
#include <tinyxml.h>

using namespace std;

// Ranges of the paramters in levels
enum Level {LV1, LV2, LV3};

// robot params structure
struct RobotParam{
	std::vector<float> vel_x{0,0,0};
	std::vector<float> vel_x_back{0,0,0};
	std::vector<float> vel_y{0,0,0};
	std::vector<float> ang_vel{0,0,0};
	std::vector<float> acc_x{0,0,0};
	std::vector<float> acc_y{0,0,0};
	std::vector<float> plan_horz{0,0,0};
	std::vector<float> band{0,0,0};
};

// human params structure
struct HumanParam{
	std::vector<float> radius{0,0,0};
	std::vector<float> vel_x{0,0,0};
	std::vector<float> vel_x_nominal{0,0,0};
	std::vector<float> vel_x_back{0,0,0};
	std::vector<float> vel_y{0,0,0};
	std::vector<float> ang_vel{0,0,0};
	std::vector<float> fov{0,0,0};
	std::vector<float> band{0,0,0};

};

// social params structure
struct SocialNorm{
	std::vector<float> safety{0,0,0};
	std::vector<float> visibility{0,0,0};
	std::vector<float> passby{0,0,0};
	std::vector<float> invis_humans{0,0,0};
};

// Data strtcure to include all cohan params to tune
struct CoHANParams{
	RobotParam RParam;
	HumanParam HParam;
	SocialNorm SNorm;
};

// Data structure to define the params in terms of levels
struct ParamList{
	string type;
	std::vector<Level> params;
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

	// Updates the parameters on server based on the ParamList given
	bool setParams(Context context);

	// Returns the existing params formatted as ParamList
	ParamList getParams(string type="all");

private:
	// Reads the params and contexts from the xml file
	bool readXMLData();

	// Stores the params from the xml
	CoHANParam cohan_params_;

	//Stores the contexts from the xml
	std::vector<ParamList> contexts_;
	std::vector<ParamList> tasks_;
	std::vector<ParamList> humans_;

	string map_name_;
	string data_file_name_;
	TiXmlDocument* doc_;

	//Params list
	std::vector<string> robot_param_names_{"max_vel_x", "max_vel_x_backwards", "max_vel_y", "max_vel_theta", "acc_lim_x",
																				"acc_lim_y", "max_global_plan_lookahead_dist", "weight_via_point"};
	std::vector<string> human_param_names_{"agent_radius --> check this again", "max_agent_vel_x", "nominal_agent_vel_x", "max_agent_vel_x_backwards", "max_agent_vel_y", "max_agent_vel_theta",
																			  "fov--> how can you use this?", "weight_agent_via_point"};
};

#endif
