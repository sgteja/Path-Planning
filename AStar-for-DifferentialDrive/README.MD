# A-Star on Differential Drive

The A-star code is used to generate a path for differential drive robots and this is tested on turtle-bot. To test it on a turtle-bot, I used ROS and Gazebo simulator. The videos are shown [here](https://github.com/sgteja/Path-Planning/tree/master/AStar-for-DifferentialDrive/Videos) \
This file structure needs to be maintained to run simulation of turtle-bot in ROS.


--> File structure

	|--Project 3 
		|
		|--README.txt
		|--rrl_world.map
		|--Sim_vid
			|
			|--1.mp4
			|--2.mp4
			|--3.mp4
		|--tbot
			|--kobuki
			|--rocon
			|--turtlebot
			|--controller
				|
				|--src
					|
					|--control
						|
						|--src
							|
							|--Astar.py
							|--main.py
							|--Map.py
							|--tbot_vel.py
							|--path.pkl
							|--Map.npy


The 'Map.npy' is the numpy 2D array which is the size of the map given in centrimetre scale(cm). So the size of the map is 1010 X 1110. This has value 255 at the empty spaces, 125 around the obstacles which is clearance + robot diameter (0.2m and 0.354m) and 0 for the obstacles. 

The 'main.py' file is used to take the inputs from the user about the start and the goal points, weight for the A* which is by default set to 1 and a boolean to show the animation while processing or not. There will be a display of final path after the algorithm finds one. 

By running this a pickle file named 'path.pkl' is generated which consists of the dictonary containing the information needed to be passed on to the Turtlebot for simulation. 

-->For simulation:
At the end of the A* algorithm it prints the coordinates of the points with respect to the frame in Gazebo, which are the starting postion for the Turtlebot. These are to be given in the kobuki.launch.xml file located at "/tbot/turtlebot/src/turtlebot_simulator/turtlebot_gazebo/launch" in the following line for x and y values
"args="-x -4.84 -y -3.96 -unpause -urdf -param robot_description -model mobile_base"/>"

Run Gazebo using the command "roslaunch turtlebot_gazebo turtlebot_world.launch", but before this as the default location of the map is directed according to my file structure need to change. Instead run the following command
"roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=(The address to the directory containing rrl_map.world)"

Then we need to publish the velocities, this can be done by running the python script named "tbot_vel.py". But make sure to change the path to the "path.pkl" file. 
"rosrun control tbot_vel.py"

The Final simulation result is as follows: \
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/ci5iAQYvzLE/0.jpg)](https://www.youtube.com/watch?v=ci5iAQYvzLE)



References

1.https://web2.qatar.cmu.edu/~gdicaro/16311/slides/start-with-ros.pdf \
2.http://gazebosim.org/tutorials?tut=ros_roslaunch \
3.http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation
