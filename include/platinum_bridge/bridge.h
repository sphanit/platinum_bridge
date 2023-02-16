#ifndef BRIDGE
#define BRIDGE

#include "ros/ros.h"
#include <string>
#include <vector>
#include <tinyxml.h>

using namespace std;

enum Level{LV1, LV2, LV3, LV4};


struct RobotParam{
	double vel;
	double ang_vel;
	double acc;
	double plan_horz;
	double band;
};

struct HumanParam{
	double radius;
	double vel;
	double ang_vel;
	double fov;
	double band;

};

struct SocialNorm{
	double safety;
	double visibility;
	double passby;
	double invis_humans;
};

struct CoHANParam{

	RobotParam RParam;
	HumanParam HParam;
	SocialNorm SNorm;
};

class PlatinumToCohan{
public:
	bool setParams();
	void getParams();

private:

};

#endif