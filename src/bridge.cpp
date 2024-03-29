#include <platinum_bridge/bridge.h>
#define NS "/move_base/HATebLocalPlannerROS/"
#define TTG_TOPIC "/move_base/HATebLocalPlannerROS/time_to_goal"
#define ParamFile "params.xml"
#define ContextFile "hospital.xml"
#define GetTokenTopic "/roxanne/acting/dispatching"
#define TokenFeedbackTopic "/roxanne/acting/feedback"

PlatinumToCohan::PlatinumToCohan(bool set_params, string log_name, bool continuous) : MB_action_client("move_base", true){
  string param_path_ = ros::package::getPath("platinum_bridge") + "/params/hospital/" + ParamFile;
  string context_path_ = ros::package::getPath("platinum_bridge") + "/maps/hospital/" + ContextFile;
  string log_file_path = ros::package::getPath("platinum_bridge") + "/logs/"+log_name+".txt";
  log_file_.open(log_file_path);
  log_file_ << "LOG STARTS : " << ros::Time::now() << endl;
  map_name_ = "hospital";


  // Loading the xml file
  param_doc_ = new TiXmlDocument(param_path_);

  if(!param_doc_->LoadFile())
    ROS_ERROR("Failed to load %s. Error : %s", param_path_.c_str(), param_doc_->ErrorDesc());
  else
    ROS_INFO("Params file loaded");

  context_doc_ = new TiXmlDocument(context_path_);

  if(!context_doc_->LoadFile())
    ROS_ERROR("Failed to load %s. Error : %s", context_path_.c_str(), context_doc_->ErrorDesc());
  else
    ROS_INFO("Contexts file loaded");

  // Check if contexts file is corresponding with map_name
  TiXmlHandle param_docHandle(context_doc_);
  TiXmlElement* l_map = param_docHandle.FirstChild("map_name").ToElement();
  string map_name_read = "";
  if(NULL != l_map->Attribute("name"))
    map_name_read = l_map->Attribute("name");
  if(map_name_read != map_name_)
    ROS_ERROR("Contexts file mismatches the map_name");
  else
    ROS_INFO("Contexts file name corresponds to map_name");

  //Initialize the param topics
  robot_param_names_ = {{"vel_x","max_vel_x"}, {"vel_x_back","max_vel_x_backwards"}, {"vel_y","max_vel_y"}, {"ang_vel","max_vel_theta"}, {"acc_x","acc_lim_x"},
                      {"acc_y","acc_lim_y"}, {"plan_horz","max_global_plan_lookahead_dist"}, {"band","weight_viapoint"}};

  human_param_names_ = {{"radius","agent_radius"}, {"vel_x","max_agent_vel_x"}, {"vel_x_nominal","nominal_agent_vel_x"}, {"vel_x_back","max_agent_vel_x_backwards"},
	                    {"vel_y","max_agent_vel_y"}, {"ang_vel","max_agent_vel_theta"}, {"fov","fov"}, {"band","weight_agent_viapoint"}};

  social_param_names_ = {{"safety","min_agent_robot_dist"}, {"visibility","visibility_cost_threshold"}, {"passby","rel_vel_cost_threshold"}, {"invis_humans","invisible_human_threshold"}};

  // Extract Information from files
  this->readParamXML();
  this->readContextXML();

  //Subscribe to the task planner
  ros::NodeHandle nh;
  get_context_ = nh.subscribe(GetTokenTopic, 1, &PlatinumToCohan::setContext, this);
  r_odom_sub_ = nh.subscribe("/odom", 1, &PlatinumToCohan::robotCB, this);
  ttg_sub_ = nh.subscribe(TTG_TOPIC, 1, &PlatinumToCohan::ttgCB, this);
  get_goal_srv_ = nh.serviceClient<platinum_bridge::getGoal>("/mongodb_goals/get_goal");
  send_feedback_token_ = nh.advertise<roxanne_rosjava_msgs::TokenExecutionFeedback>(TokenFeedbackTopic,10);

  for(auto iter = human_triggers.begin();iter!=human_triggers.end();iter++){
    names_humans.push_back(iter->first);
  }


  if(names_humans.size() >= 2){
    h1_odom_sub_ = nh.subscribe("/"+names_humans[0]+"/odom", 1, &PlatinumToCohan::human1CB, this);
    h2_odom_sub_ = nh.subscribe("/"+names_humans[1]+"/odom", 1, &PlatinumToCohan::human2CB, this);
    h1_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/"+names_humans[0]+"/move_base_simple/goal",10);
    h2_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/"+names_humans[1]+"/move_base_simple/goal",10);
  }
  else{
    h1_odom_sub_ = nh.subscribe("/human5/odom", 1, &PlatinumToCohan::human1CB, this);
    h2_odom_sub_ = nh.subscribe("/human6/odom", 1, &PlatinumToCohan::human2CB, this);
    h1_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/human5/move_base_simple/goal",10);
    h2_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/human6/move_base_simple/goal",10);
  }

  r_odom_set = h1_odom_set = h2_odom_set = false;
  start_logging_ = false;
  log_human_ = 0;

  remaining_time_to_goal = 0;
  time_to_goal = 999;
  last_recorded = ros::Time::now();

  prev_token_exist = false;
  token_updated = false;
  goal_cancelled = false;

  continuous_ = continuous;
  if(continuous_)
    ROS_INFO("Continuous goal update active!");
  if(!continuous_)
    ROS_INFO("Discrete goal update active!");

  set_params_ = set_params;
  if(set_params_)
    ROS_INFO("setParams active!");
  if(!set_params_)
    ROS_INFO("setParams disabled.");

  ROS_INFO("Waiting for the move_base action server...");
  MB_action_client.waitForServer();


  ROS_INFO("Initialized!");
  ros::spin();
}

PlatinumToCohan::~PlatinumToCohan(){
  delete param_doc_;
  delete context_doc_;
}

bool PlatinumToCohan::readContextXML(){
  TiXmlHandle docHandle(context_doc_);

  // Extracting param ranges
  TiXmlElement* l_param = docHandle.FirstChild("tasks").FirstChild("task").ToElement();

  while(l_param){
    if(NULL != l_param->Attribute("type")){
      string t_name_ = l_param->Attribute("type");
      TiXmlElement* l_plist = l_param->FirstChildElement("param");
      NamedLevel tmp;
      while(l_plist){
        string name_ = l_plist->Attribute("name");
        tmp[name_] = this->toLevel(l_plist->Attribute("level"));
        tasks_[t_name_] = tmp;
        l_plist = l_plist->NextSiblingElement("param");
      }
    }
    l_param = l_param->NextSiblingElement("task");
  }

  l_param = docHandle.FirstChild("humans").FirstChild("human").ToElement();

  while(l_param){
    if(NULL != l_param->Attribute("type")){
      string t_name_ = l_param->Attribute("type");
      TiXmlElement* l_plist = l_param->FirstChildElement("param");
      NamedLevel tmp;
      while(l_plist){
        string name_ = l_plist->Attribute("name");
        tmp[name_] = this->toLevel(l_plist->Attribute("level"));
        humans_[t_name_] = tmp;
        l_plist = l_plist->NextSiblingElement("param");
      }
    }
    l_param = l_param->NextSiblingElement("human");
  }

  l_param = docHandle.FirstChild("contexts").FirstChild("context").ToElement();

  while(l_param){
    if(NULL != l_param->Attribute("type")){
      string t_name_ = l_param->Attribute("type");
      TiXmlElement* l_plist = l_param->FirstChildElement("param");
      NamedLevel tmp;
      while(l_plist){
        string name_ = l_plist->Attribute("name");
        tmp[name_] = this->toLevel(l_plist->Attribute("level"));
        contexts_[t_name_] = tmp;
        l_plist = l_plist->NextSiblingElement("param");
      }
    }
    l_param = l_param->NextSiblingElement("context");
  }

  l_param = docHandle.FirstChild("trigger_humans").FirstChild("human").ToElement();

  while(l_param){
    if(NULL != l_param->Attribute("name")){
      string t_name_ = l_param->Attribute("name");
      string trigger = l_param->Attribute("trigger");
      string h_goal = l_param->Attribute("goal");
      vector<string> tmp{trigger,h_goal};
      human_triggers[t_name_].push_back(tmp);
    }
    l_param = l_param->NextSiblingElement("human");
  }

}

bool PlatinumToCohan::readParamXML(){
  TiXmlHandle docHandle(param_doc_);

  // Extracting param ranges for robot
  TiXmlElement* l_param = docHandle.FirstChild("params").FirstChild("r_params").FirstChild("param").ToElement();

  while(l_param){
    if(NULL != l_param->Attribute("name")){
      string name_ = l_param->Attribute("name");
      string pstr = l_param->Attribute("ranges");
      string delimiter = ", ";
      cohan_params_.RParam[name_] = this->toChars(pstr, delimiter);
    }
    l_param = l_param->NextSiblingElement("param");
  }

  l_param = docHandle.FirstChild("params").FirstChild("h_params").FirstChild("param").ToElement();

  while(l_param){
    if(NULL != l_param->Attribute("name")){
      string name_ = l_param->Attribute("name");
      string pstr = l_param->Attribute("ranges");
      string delimiter = ", ";
      cohan_params_.HParam[name_] = this->toChars(pstr, delimiter);
    }
    l_param = l_param->NextSiblingElement("param");

  }

  l_param = docHandle.FirstChild("params").FirstChild("s_norms").FirstChild("param").ToElement();

  while(l_param){
    if(NULL != l_param->Attribute("name")){
      string name_ = l_param->Attribute("name");
      string pstr = l_param->Attribute("ranges");
      string delimiter = ", ";
      cohan_params_.SNorm[name_] = this->toChars(pstr, delimiter);
      }
    l_param = l_param->NextSiblingElement("param");
  }

}

vector<Level> PlatinumToCohan::toLevels(string s, string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<Level> res;
    Level lv;

    while ((pos_end = s.find(delimiter, pos_start)) != string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        if(token == "l1"){
          lv = LV1;
        }
        else if(token == "l2"){
          lv = LV2;
        }
        else if(token == "l3"){
          lv = LV3;
        }
        res.push_back(lv);
    }
    token = s.substr(pos_start);
    if(token == "l1"){
      lv = LV1;
    }
    else if(token == "l2"){
      lv = LV2;
    }
    else if(token == "l3"){
      lv = LV3;
    }
    res.push_back(lv);

    return res;
}

Level PlatinumToCohan::toLevel(string s){
  Level lv;

  if(s=="l1")
    lv = LV1;
  else if(s=="l2")
    lv = LV2;
  else if(s=="l3")
    lv = LV3;

  return lv;
}

vector<string> PlatinumToCohan::toChars(string s, string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<string> res;
    while ((pos_end = s.find(delimiter, pos_start)) != string::npos) {
        token = s.substr(pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back(token);
    }
    res.push_back (s.substr(pos_start));
    return res;
}

vector<float> PlatinumToCohan::toFloats(string s, string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<float> res;
    while ((pos_end = s.find(delimiter, pos_start)) != string::npos) {
        token = s.substr(pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back(stof(token));
    }
    res.push_back (stof(s.substr(pos_start)));
    return res;
}

bool PlatinumToCohan::setParams(roxanne_rosjava_msgs::TokenExecution c_token){
  // ros::NodeHandle nh;
  Context context{c_token.token.parameters[1], c_token.token.parameters[2]};

  string param_set = "\"{";

  int i = 0;
  for(auto iter = cohan_params_.RParam.begin();iter!=cohan_params_.RParam.end();iter++){
    param_set += "'"+robot_param_names_[iter->first] + "':";
    param_set += iter->second[int(tasks_[context.task][iter->first])]+", ";
    i++;
  }

  i = 0;
  for(auto iter = cohan_params_.HParam.begin();iter!=cohan_params_.HParam.end();iter++){
    if(human_param_names_[iter->first] == "agent_radius")
      continue;
    param_set += "'"+human_param_names_[iter->first] + "':";
    param_set += iter->second[int(humans_[context.human_type][iter->first])]+", ";
    i++;
  }

  i = 0;
  for(auto iter = cohan_params_.SNorm.begin();iter!=cohan_params_.SNorm.end();iter++){
    param_set += "'"+social_param_names_[iter->first] + "':";
    param_set += iter->second[int(contexts_[context.task+"_"+context.human_type][iter->first])]+", ";
    i++;
  }
  param_set+="}\"";
  string command = "rosrun dynamic_reconfigure dynparam set /move_base/HATebLocalPlannerROS ";
  command+=param_set;

  while(system(command.c_str())){
    continue;
  }


  ROS_INFO("Params updated --> Task:%s Human:%s",context.task.c_str(), context.human_type.c_str());
}

void  PlatinumToCohan::setContext(const roxanne_rosjava_msgs::TokenExecution &token){
  current_token_ = token;
  ROS_INFO("New token received.");
  
  if(!prev_token_exist){
    // Call the setParams
    if(set_params_)
      this->setParams(current_token_);

    if(continuous_){
      // Send the first goal to base
      this->sendGoalToBase(current_token_);
    }
  }

  if(!continuous_){
    // Send the goal to base
    this->sendGoalToBase(current_token_);
  }

  prev_token_exist = true;
  token_updated = true;
  goal_cancelled = false;
}

void PlatinumToCohan::sendGoalToBase(roxanne_rosjava_msgs::TokenExecution c_token){
  platinum_bridge::getGoal goalsrv;
  goalsrv.request.goal_name = c_token.token.parameters[0];

  if(get_goal_srv_.call(goalsrv)){
    ROS_INFO("Got the goal from the server :x=%f, y=%f, yaw=%f",goalsrv.response.coordinates[0], goalsrv.response.coordinates[1], goalsrv.response.coordinates[2]);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = goalsrv.response.coordinates[0];
    goal.target_pose.pose.position.y = goalsrv.response.coordinates[1];
    tf2::Quaternion q;
    if(c_token.token.parameters[3]!="out")
      q.setRPY(0, 0, goalsrv.response.coordinates[2]);
    else
      q.setRPY(0, 0, (goalsrv.response.coordinates[2]+3.14));
    tf2::convert(q, goal.target_pose.pose.orientation);
    // goal.target_pose.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped h_goal;
    h_goal.header.frame_id = "map";

    int hum = 0;

    string trigger_ = c_token.token.parameters[3]+"_"+c_token.token.parameters[0];

    // if(trigger_==human_triggers["human5"][0]){
    //   h_goal.pose.position.x = 12;
    //   h_goal.pose.position.y = 11.5;
    //   h_goal.pose.orientation.w = 0.707;
    //   h_goal.pose.orientation.z = 0.707;
    //   h1_goal_pub_.publish(h_goal);
    //   hum = 1;
    // }
    //
    // if(trigger_==human_triggers["human6"][0]){
    //   h_goal.pose.position.x = 9.5;
    //   h_goal.pose.position.y = 5.0;
    //   h_goal.pose.orientation.w = 0.707;
    //   h_goal.pose.orientation.z = -0.707;
    //   h2_goal_pub_.publish(h_goal);
    //   hum=2;
    // }

    for(int i=0;i<names_humans.size();i++){
      for(int j=0;j<human_triggers[names_humans[i]].size();j++){
        if(trigger_==human_triggers[names_humans[i]][j][0]){
          string delimiter = " ";
          auto tmp_goal = this->toFloats(human_triggers[names_humans[i]][j][1], delimiter);
          h_goal.pose.position.x = tmp_goal[0];
          h_goal.pose.position.y = tmp_goal[1];
          tf2::Quaternion q1;
          q1.setRPY(0, 0, tmp_goal[2]);
          tf2::convert(q1, h_goal.pose.orientation);

          if(i==0){
            h1_goal_pub_.publish(h_goal);
            hum = 1;
          }

          if(i==1){
            h2_goal_pub_.publish(h_goal);
            hum = 2;
          }
        }
      }
    }

    // Start logging
    log_file_ << "Phase : " << c_token.tokenId <<" "<<c_token.token.parameters[1] << " " << c_token.token.parameters[2] << endl;
    this->startLogging(hum);

    // Need boost::bind to pass in the 'this' pointer
    MB_action_client.sendGoal(goal,boost::bind(&PlatinumToCohan::doneCb, this, _1, _2),
                              boost::bind(&PlatinumToCohan::activeCb, this),
                              MoveBaseClient::SimpleFeedbackCallback()
                              // boost::bind(&PlatinumToCohan::feedbackCb, this, _1)
                            );

  }
  else{
    ROS_ERROR("Goal cannot be retrieved!!");
  }

  // Checking the expected time to goal
  remaining_time_to_goal = c_token.token.duration[1];
  last_recorded = ros::Time::now();
}

void PlatinumToCohan::ttgCB(const std_msgs::Float32& ttg){
  time_to_goal = ttg.data;
  
  double dt_diff = (ros::Time::now() - last_recorded).toSec();
  remaining_time_to_goal = remaining_time_to_goal - dt_diff;

  // std::cout << "remaining_time_to_goal " << remaining_time_to_goal << std::endl;
  // std::cout << "time_to_goal " << time_to_goal << std::endl;

  if(!goal_cancelled){

    if(remaining_time_to_goal < time_to_goal){
      MB_action_client.cancelAllGoals();
      token_feedback_.tokenId = current_token_.tokenId;
      token_feedback_.code = 1;
      send_feedback_token_.publish(token_feedback_);
      start_logging_ = false;
      time_to_goal = 99;
      std::cout << "Cancelled the goal" << std::endl;
      goal_cancelled = true;
    }

    if(token_updated){
      
      double tc = 0.5;
      if(set_params_)
        tc = 2.5;

      if(current_token_.next.size()!=0){
        if(time_to_goal < tc){
          roxanne_rosjava_msgs::TokenExecution c_token; 
          c_token.tokenId = current_token_.tokenId + 1;
          c_token.token = current_token_.next[1];
          if(set_params_)
            this->setParams(c_token);
          token_updated = false;

          if(continuous_){
            start_logging_ = false;
            this->sendGoalToBase(c_token);
            token_feedback_.tokenId = current_token_.tokenId;
            token_feedback_.code = 0;
            send_feedback_token_.publish(token_feedback_);
            time_to_goal = 99;
          }
        }
      }
      else
        prev_token_exist = false;
    }

    last_recorded = ros::Time::now();
  }
}

void PlatinumToCohan::doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){
  if(!goal_cancelled){
    ROS_INFO("Goal Reached!");
    token_feedback_.tokenId = current_token_.tokenId;
    token_feedback_.code = 0;
    send_feedback_token_.publish(token_feedback_);
    start_logging_ = false;
    time_to_goal = 99;
  }
}

void PlatinumToCohan::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
  ROS_INFO("Got Feedback frame");
}

void PlatinumToCohan::activeCb(){
  ROS_INFO("Goal execution started.");
}

void PlatinumToCohan::startLogging(int human){
  start_logging_ = true;
  log_human_= human;
}


void PlatinumToCohan::robotCB(const nav_msgs::Odometry::ConstPtr& msg){
  robot_odom = *msg;
  r_odom_set = true;

  if(start_logging_){
    if(log_human_ == 1 && h1_odom_set == true){
      h1_odom_set = false;

      auto q = human1_odom.pose.pose.orientation;
      double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      double theta_yaw = atan2(siny_cosp, cosy_cosp);

      log_file_ << ros::Time::now() << " : H " << human1_odom.pose.pose.position.x << " " << human1_odom.pose.pose.position.y  << " " << theta_yaw << endl;
      log_file_ << ros::Time::now() << " : LOG VEL_H " << std::to_string(sqrt(pow(human1_odom.twist.twist.linear.x,2) + pow(human1_odom.twist.twist.linear.y,2))) << endl;

      q = robot_odom.pose.pose.orientation;
      siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      theta_yaw = atan2(siny_cosp, cosy_cosp);

      log_file_ << ros::Time::now() << " : R " << robot_odom.pose.pose.position.x << " " << robot_odom.pose.pose.position.y  << " " << theta_yaw << endl;
      log_file_ << ros::Time::now() << " : LOG VEL_R " << std::to_string(sqrt(pow(robot_odom.twist.twist.linear.x,2) + pow(robot_odom.twist.twist.linear.y,2))) << endl;

      string costs_ = this->computeTTC(human1_odom);

      // log_file_ << costs_ << endl;
    }

    else if(log_human_ == 2 && h2_odom_set == true){
      h2_odom_set = false;

      auto q = human2_odom.pose.pose.orientation;
      double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      double theta_yaw = atan2(siny_cosp, cosy_cosp);

      log_file_ << ros::Time::now() << " : H " << human2_odom.pose.pose.position.x << " " << human2_odom.pose.pose.position.y  << " " << theta_yaw << endl;
      log_file_ << ros::Time::now() << " : LOG VEL_H " << std::to_string(sqrt(pow(human2_odom.twist.twist.linear.x,2) + pow(human2_odom.twist.twist.linear.y,2))) << endl;

      q = robot_odom.pose.pose.orientation;
      siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      theta_yaw = atan2(siny_cosp, cosy_cosp);

      log_file_ << ros::Time::now() << " : R " << robot_odom.pose.pose.position.x << " " << robot_odom.pose.pose.position.y  << " " << theta_yaw << endl;
      log_file_ << ros::Time::now() << " : LOG VEL_R " << std::to_string(sqrt(pow(robot_odom.twist.twist.linear.x,2) + pow(robot_odom.twist.twist.linear.y,2))) << endl;

      string costs_ = this->computeTTC(human2_odom);

      // log_file_ << costs_ << endl;
    }

    else{

      auto q = robot_odom.pose.pose.orientation;
      double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      double theta_yaw = atan2(siny_cosp, cosy_cosp);

      log_file_ << ros::Time::now() << " : R " << robot_odom.pose.pose.position.x << " " << robot_odom.pose.pose.position.y  << " " << theta_yaw << endl;
      log_file_ << ros::Time::now() << " : LOG VEL_R " << std::to_string(sqrt(pow(robot_odom.twist.twist.linear.x,2) + pow(robot_odom.twist.twist.linear.y,2))) << endl;

    }
  }
}

void PlatinumToCohan::human1CB(const nav_msgs::Odometry::ConstPtr& msg){
    human1_odom = *msg;
    h1_odom_set = true;
}

void PlatinumToCohan::human2CB(const nav_msgs::Odometry::ConstPtr& msg){
    human2_odom = *msg;
    h2_odom_set = true;
}

string PlatinumToCohan::computeTTC(nav_msgs::Odometry human_odom){
  double ttc_ = -1.0; // ttc infinite
	double c_danger = -1;
	double c_passby = -1;
  string msg_log_;

	geometry_msgs::Pose2D C; // robot human relative position
	C.x = human_odom.pose.pose.position.x - robot_odom.pose.pose.position.x;
	C.y = human_odom.pose.pose.position.y - robot_odom.pose.pose.position.y;
	double C_sq = C.x*C.x + C.y*C.y; // dot product C.C, distance robot human

	// Previous
	// double robot_inflated_radius = robot_radius_*C_sq/dist_radius_inflation_;

	// New
	double robot_inflated_radius = 0.47;
	double radius_sum = 0.31 + robot_inflated_radius;
	double radius_sum_sq_ = radius_sum*radius_sum;

	if(C_sq <= radius_sum_sq_) // already touching
		ttc_ = 0.0;
	else
	{
		geometry_msgs::Twist V; // relative velocity human to robot
		V.linear.x = robot_odom.twist.twist.linear.x - human_odom.twist.twist.linear.x;
		V.linear.y = robot_odom.twist.twist.linear.y - human_odom.twist.twist.linear.y;

		double C_dot_V = C.x*V.linear.x + C.y*V.linear.y;

		if(C_dot_V > 0) // otherwise ttc infinite
		{
			double V_sq = V.linear.x*V.linear.x + V.linear.y*V.linear.y;
			double f = (C_dot_V * C_dot_V) - (V_sq * (C_sq - radius_sum_sq_));
			if(f > 0) // otherwise ttc infinite
			{
				ttc_ = (C_dot_V - sqrt(f)) / V_sq;
			}

			else
			{
				double g = sqrt(V_sq*C_sq - C_dot_V*C_dot_V);
				if((g - (sqrt(V_sq)*radius_sum))>0.1)
				{
					c_passby = sqrt(V_sq/C_sq)*(g/(g - (sqrt(V_sq)*radius_sum)));
				}
			}
		}
	}

	if(ttc_ != -1)
	{
		// ROS_INFO("HBM: TTC = %f", ttc_);
		if(abs(robot_odom.twist.twist.linear.x)>0.001 || abs(robot_odom.twist.twist.linear.y)>0.001){
		msg_log_ = std::to_string(ros::Time::now().toSec()) + " : HUMAN_MODEL TTC " + std::to_string(ttc_) + " " + "\n";
		}

		if(ttc_ > 0)
		{
			c_danger = 1/ttc_;
		}
	}

	if(abs(robot_odom.twist.twist.linear.x)>0.001 || abs(robot_odom.twist.twist.linear.y)>0.001){
    msg_log_ =  std::to_string(ros::Time::now().toSec()) + " : HUMAN_MODEL C_DANGER " + std::to_string(c_danger) + "\n";

		msg_log_ += std::to_string(ros::Time::now().toSec()) + " : HUMAN_MODEL C_PASSBY " + std::to_string(c_passby) + "\n";
	}

  return msg_log_;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "platinum_bridge");

  if(argc<4){
    PlatinumToCohan pc_bridge(true, "log.txt", false);
      // ros::spin();
  }
  else{
    string set_param = argv[1];
    string log_name_ = argv[2];
    string continuous = argv[3];
    if(set_param == "true"){
      if(continuous == "true")
        PlatinumToCohan pc_bridge(true, log_name_, true);
      else
        PlatinumToCohan pc_bridge(true, log_name_, false);
    }
    else{
      if(continuous == "true")
        PlatinumToCohan pc_bridge(false, log_name_, true);
      else
        PlatinumToCohan pc_bridge(false, log_name_, false);
    }

    }

  return 0;
}
