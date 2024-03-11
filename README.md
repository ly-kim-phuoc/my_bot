## Robot Package for Mobile Robot using ROS
### How to use :
* cd dev_ws : change directory to workspace
* source install/setup.bash : source to use ros2
* rviz2 : open Rviz
* ros2 launch my_bot launch_sim.launch.py world:=/home/phuoc/dev_ws/worlds/room.world use_sim_time:=true : publish bot and open gazebo to simulate
* ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=true  : turn on slam
* ros2 run teleop_twist_keyboard teleop_twist_keyboard : control robot with keyboard
* ros2 run nav2_map_server map_saver_cli -f /home/phuoc/dev_ws/src/my_bot/maps/map
* ros2 launch my_bot map.launch.py : publish pre-generated map

*In case of error :*
* gazebo -s libgazebo_ros_init.so -s libgazebo_ros_factory.so myworld.world : open gazebo
* ros2 launch my_bot rsp.launch.py use_sim_time:=true : publish bot
* ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity robot_name : spawn bot to gazebo

### Wiring :
Left wheel :
* In 1 - D6
* In 2 - D10
* C1 - D3
* C2 - D2

Right wheel :
* In 3 - D9
* In 4 - D5
* C1 - A4
* C2 - A5
