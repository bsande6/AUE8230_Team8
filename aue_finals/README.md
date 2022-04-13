
Steps to run final project code in gazebo

export GAZEBO_MODEL_PATH=~/catkin_ws/src/AUE8230_Team8/aue_finals/models/:~/catkin_ws/src/turtlebot3>

roslaunch aue_finals turtlebot3_autonomy_final.launch 

The launch file should open a mini window which allows us to enter a key to switch nodes

Then in new terminal run

rosrun aue_finals turtlebot_master.py

To start the wall following make sure the keyboard box is open and press 'w'. 




