## Specific linefollowing gazebo instructions

#Videos available at 
https://drive.google.com/drive/folders/1D058jfID2LoCvej8cDOqqjm2bv-FjXuq

#Run line follow in simulation

# Run the following in new terminals

	roslaunch turtlebot3_gazebo turtlebot3_follow_line.launch 

	rosrun turtlebot3_teleop turtlebot3_teleop_key 

# Use teleop to move turtle bot into line facing the forward on the track then run the following command
# in a new terminal

	rosrun assignment6_trackingandfollowing follow_line_step_hsv.py



## Specific linefollowing realworld instructions

## Modify pc bashrc to use ifconfig address 

# New terminal

	ssh ubuntu@192.168.0.170

	roslaunch turtlebot3_bringup turtlebot3_robot.launch

# New terminal
	
	ssh ubuntu@192.168.0.170
	
	roslaunch raspicam_node camerav2_1280x960.launch enable_raw:=true

# Run the following to test the camera
	
	rqt_image_view

# Run the following for lane following in real world

	rosrun assignment6_trackingandfollowing real_follow_line_step_hsv.py



## ROS April Tag

# New terminal

	ssh ubuntu@192.168.0.170

	roslaunch turtlebot3_bringup turtlebot3_robot.launch

# New terminal
	
	ssh ubuntu@192.168.0.170
	
	roslaunch raspicam_node camerav2_1280x960.launch enable_raw:=true


# New terminal

	roslaunch apriltag_ros continuous_detection.launch


# New terminal

	rqt_image_view





