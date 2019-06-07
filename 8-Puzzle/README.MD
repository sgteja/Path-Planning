Project-1
8 Puzzle
The program is written and tested using python 2.7 version.
After the program is executed it asks for the following inputs as follows:
The initial configuration to start with given in the format separated by commas in a flat format.
Then it displays the entered matrix and asks to confirm by typing Yes or No otherwise.
It displays the default goal matrix and asks to confirm, if entered No then it waits for the input of the goal matrix same as the format of the initial matrix.
Then asks for the maximum number of parent nodes to be visited.
Awaits for a reply whether or not to print the NodePath steps after solving.
After these steps the code will be executed. Every input given has a default value set and that can be used by just pressing enter without giving any data. This code first checks for solvability after which it proceeds to solve the puzzle.

The output matrices are saved in the text files as instructed in the ‘./OutputFiles’ folder in the same directory where the code is running from. The file tree structure is as follows,
|-script.py
|-OutpuFiles
	|
	|-NodePath.txt
	|-Nodes.txt
	|-NodesInfo.txt	

By 
Gnyana Teja Samudrala
UID:115824737
