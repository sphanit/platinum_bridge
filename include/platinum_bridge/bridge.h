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
#include <tf2/impl/utils.h>
#include <tf2/convert.h>
#include "roxanne_rosjava_msgs/TokenExecution.h"
#include "roxanne_rosjava_msgs/TokenExecutionFeedback.h"
#include "platinum_bridge/getGoal.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <fstream>
#include <math.h>


using namespace std;

// Ranges of the paramters in levels
enum Level {LV1, LV2, LV3};

typedef map<string, vector<string>> NamedList;
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
  PlatinumToCohan(bool set_params, string log_name);

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
  vector<string> toChars(string s, string delimiter);
  vector<float> toFloats(string s, string delimiter);
  void setContext(const roxanne_rosjava_msgs::TokenExecution &token);
  void sendGoalToBase();
  void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  void activeCb();
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
  void startLogging(int human);

  string computeTTC(nav_msgs::Odometry human_odom);
  void robotCB(const nav_msgs::Odometry::ConstPtr& msg);
  void human1CB(const nav_msgs::Odometry::ConstPtr& msg);
  void human2CB(const nav_msgs::Odometry::ConstPtr& msg);


  // Stores the params from the xml
  CoHANParams cohan_params_;

  //Stores the contexts from the xml
  LevelMap contexts_;
  LevelMap tasks_;
  LevelMap humans_;
  map<string, vector<string>> human_triggers;
  vector<string> names_humans;

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
  ros::Subscriber get_context_,r_odom_sub_, h1_odom_sub_, h2_odom_sub_;
  ros::ServiceClient get_goal_srv_;
  ros::Publisher send_feedback_token_, h1_goal_pub_, h2_goal_pub_;

  //MoveBase action client
  MoveBaseClient MB_action_client;

  //Logging the data
  ofstream log_file_;
  nav_msgs::Odometry robot_odom, human1_odom, human2_odom;
  bool r_odom_set, h1_odom_set, h2_odom_set, start_logging_;
  int log_human_;
  bool set_params_;

};

#endif
