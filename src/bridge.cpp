#include <platinum_bridge/bridge.h>
#define NS "/move_base/HATebLocalPlannerROS/"
#define ParamFile "params.xml"
#define ContextFile "hospital.xml"

PlatinumToCohan::PlatinumToCohan(){
  string param_path_ = ros::package::getPath("platinum_bridge") + "/params/hospital/" + ParamFile;
  string context_path_ = ros::package::getPath("platinum_bridge") + "/maps/hospital/" + ContextFile;
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
											{"acc_y","acc_lim_y"}, {"plan_horz","max_global_plan_lookahead_dist"}, {"band","weight_via_point"}};

  human_param_names_ = {{"radius","agent_radius"}, {"vel_x","max_agent_vel_x"}, {"vel_x_nominal","nominal_agent_vel_x"}, {"vel_x_back","max_agent_vel_x_backwards"}, 
	                    {"vel_y","max_agent_vel_y"}, {"ang_vel","max_agent_vel_theta"}, {"fov","fov"}, {"band","weight_agent_via_point"}};

  social_param_names_ = {{"safety","min_agent_robot_dist"}, {"visibility","visibility_cost_threshold"}, {"passby","rel_vel_cost_threshold"}, {"invis_humans","invisible_human_threshold"}};	

  // Extract Information from files
	this->readParamXML();
	this->readContextXML();
  
  // Call the setParams
  Context cnt;
  cnt.task = "social";
  cnt.human_type = "fragile";
  this->setParams(cnt);

  ROS_INFO("Initialized");

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

      while(l_plist){
        string name_ = l_plist->Attribute("name");
        NamedLevel tmp;
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

      while(l_plist){
        string name_ = l_plist->Attribute("name");
        NamedLevel tmp;
        tmp[name_] = this->toLevel(l_plist->Attribute("level"));        
        humans_[t_name_] = tmp;
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

      while(l_plist){
        string name_ = l_plist->Attribute("name");
        NamedLevel tmp;
        tmp[name_] = this->toLevel(l_plist->Attribute("level"));        
        contexts_[t_name_] = tmp;
        l_plist = l_plist->NextSiblingElement("param");
      }
    }
		l_param = l_param->NextSiblingElement("task");
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
      cohan_params_.RParam[name_] = this->toFloats(pstr, delimiter);
    }
		l_param = l_param->NextSiblingElement("param");
  }

	l_param = docHandle.FirstChild("params").FirstChild("h_params").FirstChild("param").ToElement();
  
  while(l_param){
		if(NULL != l_param->Attribute("name")){
      string name_ = l_param->Attribute("name");
      string pstr = l_param->Attribute("ranges");
      string delimiter = ", ";
      cohan_params_.HParam[name_] = this->toFloats(pstr, delimiter);
    }
  	l_param = l_param->NextSiblingElement("param");
    
  }

	l_param = docHandle.FirstChild("params").FirstChild("s_norms").FirstChild("param").ToElement();

  while(l_param){
    if(NULL != l_param->Attribute("name")){
      string name_ = l_param->Attribute("name");
      string pstr = l_param->Attribute("ranges");
      string delimiter = ", ";
      cohan_params_.SNorm[name_] = this->toFloats(pstr, delimiter);
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

vector<float> PlatinumToCohan::toFloats(string s, string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<float> res;
    while ((pos_end = s.find(delimiter, pos_start)) != string::npos) {
        token = s.substr(pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back(stod(token));
    }
    res.push_back (stof(s.substr(pos_start)));
    return res;
}

bool PlatinumToCohan::setParams(Context context){
  ros::NodeHandle nh;

  int i = 0;
  for(auto iter = cohan_params_.RParam.begin();iter!=cohan_params_.RParam.end();iter++){
    // cout << iter->second[int(tasks_[context.task][iter->first])] << endl;
    // cout << robot_param_names_[iter->first] << endl;
    nh.setParam(NS+robot_param_names_[iter->first],iter->second[int(tasks_[context.task][iter->first])]);
    i++;
  }

  i = 0;
  for(auto iter = cohan_params_.HParam.begin();iter!=cohan_params_.HParam.end();iter++){
    nh.setParam(NS+human_param_names_[iter->first],iter->second[int(humans_[context.human_type][iter->first])]);
    i++;
  }

  i = 0;
  for(auto iter = cohan_params_.SNorm.begin();iter!=cohan_params_.SNorm.end();iter++){
    nh.setParam(NS+social_param_names_[iter->first],iter->second[int(contexts_[context.task+"_"+context.human_type][iter->first])]);
    i++;
  }

}



int main(int argc, char** argv){
  ros::init(argc, argv, "platinum_bridge");
  
  PlatinumToCohan pc_bridge;

  ros::spin();

  return 0;
}
