in 4 different terminals run the following:

ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa4 competitor_pkg:=group2
ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py
ros2 launch group2 competitor.launch.py
ros2 run group2 zoro.py


