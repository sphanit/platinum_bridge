#ifndef BRIDGE
#define BRIDGE

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <sstream>
#include <tinyxml.h>
#include <map>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "roxanne_rosjava_msgs/TokenExecution.h"
#include "roxanne_rosjava_msgs/TokenExecutionFeedback.h"
#include "platinum_bridge/getGoal.h"


using namespace std;

// Ranges of the paramters in levels
enum Level {LV1, LV2, LV3};

typedef map<string, vector<float>> NamedList;
typedef map<string, Level> NamedLevel;
typedef map<string, NamedLevel> LevelMap;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
  bool setParams();

  // Returns the existing params formatted as ParamList
  ParamList getParams(string type="all");

private:
  // Reads the params and contexts from the xml file
  bool readParamXML();
  bool readContextXML();
  vector<Level> toLevels(string s, string delimiter);
  Level toLevel(string s);
  vector<float> toFloats(string s, string delimiter);
  void setContext(const roxanne_rosjava_msgs::TokenExecution &token);
  void sendGoalToBase();
  void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  void activeCb();
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

  // Stores the params from the xml
  CoHANParams cohan_params_;

  //Stores the contexts from the xml
  LevelMap contexts_;
  LevelMap tasks_;
  LevelMap humans_;

  // Read XML handles
  string map_name_;
  TiXmlDocument* param_doc_;
  TiXmlDocument* context_doc_;

  //Params list
  map<string, string> robot_param_names_, human_param_names_, social_param_names_;

  // Roxanne token and feedback
  roxanne_rosjava_msgs::TokenExecution current_token_;
  roxanne_rosjava_msgs::TokenExecutionFeedback token_feedback_;

  //ROS COMPONENTS
  ros::Subscriber get_context_;
  ros::ServiceClient get_goal_srv_;
  ros::Publisher send_feedback_token_;

  //MoveBase action client
  MoveBaseClient MB_action_client;

};

#endif
