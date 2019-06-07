To run the program, in command window run "python main.py"
This runs the program with default values which are as follows
	--Start point		[40,130]
	--Goal point		[220,10]
	--Resolution		5
	--Robot Diameter	10
	--Clearance			0	
	In default mode, the map is generated first for the point robot and then 
	runs Dijkstra and Astar algorithm with animation showing the exploration.
	Next map is generated for rigid robot to consider it as a point robot by
	expanding the obstacles. Then applies Dijkstra and Astar algorithm.

The input to the code can be given as python argument parser, the list of arguments given are as follows
	--Start			50 60			Start point 
	--Goal			210 10			Goal point
	--Res 			2				Resolution or the grid size
	--RoboDia		5				Diameter of the robot
	--Clearance		2				Clearance to be given
	--AStar 		'T'				Whether to perform A* or not ('F')
	--Weight		2				Weight value for the weighted A*, default is 1
	--Dijk 			'T'  			Whether to perform Dijkstra or not ('F')
	--Anima 		'T' 			Whether to show the live exploration of nodes
	--checkAll 		'T'  			This if 'T' runs all the algorithms required for 									both point and rigid robot.

Example showing the statement for passing the inputs,

		"python main.py --Start 70 10 --Goal 210 10 --Res 5 --RoboDia 4 --Clearance 2
		--AStar 'T' --Weight 1 --Dijk 'F' --checkAll 'F'"

	This statement starts from (70,10) and proceeds to (210,10) using only A* algorithm for robot with 4mm diameter, maintaining a clearance of 2mm.

This program first checks if the given start and goal points are in the map and not on the obstacle, if not continues with the program or else prints the specific error. Also few points cannot be reached with a specific resolution, in this case also it stops the program and prints the error.
