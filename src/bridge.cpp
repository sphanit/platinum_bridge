


int main(int argc, char** argv)
{

  ros::init(argc, argv, "platinum_bridge");
  ros::NodeHandle nh;
  std::string global_name, relative_name, default_param;
  double vel;
  // std::vector<std::string> keys;
  // ros::param::search(keys);

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

// if (ros::param::get("relative_name", relative_name))
// {
// ...
// }

// Default value version
// ros::param::param<std::string>("default_param", default_param, "default_value");


// Setting ###############
// ros::param::set("/global_param", 5);
// ros::param::set("relative_param", "my_string");
// ros::param::set("bool_param", false);


//Checking #############

// if (ros::param::has("my_param"))
// {
//   ...
// }

// std::string param;
// ros::param::get("~private_name", param);