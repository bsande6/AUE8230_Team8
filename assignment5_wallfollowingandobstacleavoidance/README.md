Part 1) Wall Following
The launch file simply opens a world for wall following and automatically starts the turtlebot via the wall_following.py script
To perform wall following in gazebo open the wall following world via 

roslaunch assignment5_wallfollowingandobstacleavoidance  wall_following.launch 

then in a new terminal use the cmd 

rosrun assignment5_wallfollowingandobstacleavoidance wall_following.py

to start the turtlebot.

Part 2) Object Avoidance
The launch file opens a world for obstacle avoidance which contains assorted blocks scattered through the world and then automatically starts the turtlebot via the wander.py script

Open the obstacle avoidance world with 

roslaunch assignment5_wallfollowingandobstacleavoidance obstacle_avoidance.launch 



