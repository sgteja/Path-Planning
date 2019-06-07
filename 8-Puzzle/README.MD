# 8 Puzzle

|1 | 2 | 3|
|--|--|--|
|**4** | **5** | **6**|
|**7** |**8**  |**0**|

In this game the above shown state is the desired state and the starting state can be anything with the numbers misplaced, only sliding with the ```0``` tile is possible move in the game. There will be few states from which game cannot be solved, those states are checked before starting the code, this is ```solvable(state)``` function in the code.

The program is written and tested using python 2.7 version.\
After the program is executed it asks for the following inputs as follows:
- The initial configuration to start with given in the format separated by commas in a flat format.
- Then it displays the entered matrix and asks to confirm by typing Yes or No otherwise.
- It displays the default goal matrix and asks to confirm, if entered No then it waits for the input of the goal matrix same as the format of the initial matrix.
- Then asks for the maximum number of parent nodes to be visited.
- Awaits for a reply whether or not to print the NodePath steps after solving.
After these steps the code will be executed. Every input given has a default value set and that can be used by just pressing enter without giving any data. This code first checks for solvability after which it proceeds to solve the puzzle.

The output matrices are saved in the text files in the ```./OutputFiles``` folder in the same directory where the code is running from. The file tree structure is as follows,
- |-script.py
- |-OutpuFiles
	- |
	- |-NodePath.txt
	- |-Nodes.txt
	- |-NodesInfo.txt	
