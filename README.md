# Platinum Bridge
## Clone the package into the source folder of roxanne
```
cd <roxanne_ws>/src
git clone https://github.com/sphanit/platinum_bridge.git
```
## Building the package
- Before building the package, a small change has to be make in CMakeLists.txt of roxanne_rosjava_msgs. Reorder the ```catkin_package()``` and ```catkin_rosjava_setup()``` such that they appear in the following order:
```
catkin_package()

catkin_rosjava_setup()
```
- Build the package
```
cd <roxanne_ws>
catkin_make
source devel/setup.bash
```
# CoHAN
- Follow the instructions given [here](https://github.com/sphanit/cohan_planner_multi) to clone and build the CoHAN planner. Make sure you are in the branch ```main```.
- Once the planner is installed, clone the navigation configurations and launch files (branch ```multi```):
```
cd <cohan_ws>/src
git clone https://github.com/sphanit/CoHAN_Navigation.git -b multi
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
catkin build
```

# Launching CoHAN and Platinum Bridge
1. Run the roscore (optional)
```
roscore
```
2. In a separate terminal, launch cohan with the correct ```map_name``` and the number of humans ```num_agents```
```
cd <cohan_ws>
source devel/setup.bash
roslaunch cohan_navigation stage_pr2_only.launch map_name:=hospital num_agents:=6
```
3. For controlling the human agents, launch this in a separate terminal after the robot is launched. You can choose any human between 1 and 6 --> Should be same as defined in hospital.xml (goals are defined here). Example uses human5 and human6.
   
 ```
   roslaunch cohan_navigation stage_two_agents.launch ns1:=human5 ns2:=human6 num_agents:=6 map_name:=hospital
 ```
   
4. In a separate terminal, launch the bridge (also runs the goal server and loads the goals to mongodb database).    
*Make sure that mongodb server is running locally before launching the above*
```
cd <roxanne_ws>
source devel/setup.bash
roslaunch platinum_bridge roxanne_bridge.launch set_params:=true log_name:test_log continuous:=true
```
   -  ```set_params``` corresponds to dynamic parameter setting (true -> paramters are updated based on task)  
   -  ```log_name``` is the name of the log file that logs the data and metrics  
   -  when ```continuous``` param is true, the goal is updated before receiving a next token. (default -> false)  
 
5. Now run roxanne. If everything works fine, the robot should move to the locations gived by ```/roxanne/acting/dispatching``` topic. The coordinates of these locations can be modified from platinum_bridge/scripts/map1.json


