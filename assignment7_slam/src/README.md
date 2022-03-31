

## Run the following commands

	roscore

# in turtlebot terminal

	ssh pi@{IP_ADDRESS_OF_RASPBERRY_PI}

	roslaunch turtlebot3_bringup turtlebot3_robot.launch

# in remote pc

	roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

	roslaunch turtlebot3_slam turtlebot3_slam.launch

	or

	roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=karto


	roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml


# videoes named SLAM I-III and Map_Karto and Regular_Map show the SLAM action
	
	https://drive.google.com/file/d/10XXS90LW1k2q4jhdFJfS-kjbEqD3ddSS/view?usp=sharing



