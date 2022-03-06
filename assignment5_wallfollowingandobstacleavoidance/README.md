Wall Following and Obstacle Avoidance

Part 1) Wall Following

The launch file simply opens a world for wall following and automatically starts the turtlebot via the wall_following.py script. The wall following algorithm is based on a source from k-state.edu and is explained at http://faculty.salina.k-state.edu/tim/robot_prog/MobileBot/Algorithms/WallFollow.html

The algorithm utilizes lidar points to the left of the vehicle and a point at an angle of 70 degrees from the front of the turtlebot on the left side. These distances are then used to calculate the anglular command to send the turtlebot based on the desired distance from the wall and an adjustable wall_distance constant which can be changed to adjust the rate that the turtlebot will turn.

To perform wall following in gazebo open the wall following world via 

roslaunch assignment5_wallfollowingandobstacleavoidance  wall_following.launch 


Part 2) Object Avoidance
The launch file opens a world for obstacle avoidance which contains assorted blocks scattered through the world and then automatically starts the turtlebot via the wander.py script

Open the obstacle avoidance world with 

roslaunch assignment5_wallfollowingandobstacleavoidance obstacle_avoidance.launch 



