#include <bridge.h>
#define NS "/move_base/HATebLocalPlannerROS/"

PlatinumToCohan::PlatinumToCohan(){
  ros::NodeHandle nh;
}

PlatinumToCohan::~PlatinumToCohan(){
  delete doc_;
}

bool PlatinumToCohan::readXMLData(){

}















int main(int argc, char** argv)
{

  ros::init(argc, argv, "platinum_bridge");
  ros::NodeHandle nh;
  std::string global_name, relative_name, default_param;
  double vel;

  if(nh.getParam("/move_base/HATebLocalPlannerROS/max_vel_x", vel))
  {
     std::cout << vel << '\n';
     nh.setParam("/move_base/HATebLocalPlannerROS/max_vel_x", 0.3);
  }
  else{
    std::cout << "None" << '\n';
  }


  ros::spin();

  return 0;

}
