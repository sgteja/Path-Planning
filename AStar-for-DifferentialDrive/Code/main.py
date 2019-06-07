import numpy as np 
import matplotlib.pyplot as plt
import argparse
import warnings
import time
warnings.filterwarnings("ignore")

from AStar import runAStar

def booleanParser(val):
	if val.lower() == 't':
		return True
	else:
		return False

def adjPts(st,goal):

	str(st[0])[len(str(st[0]))-1] = '0'

def run():

	stPt = (stNode[0],stNode[1])
	gPt = (gNode[0],gNode[1])

	Map = np.load('Map.npy')

	if Map.shape[1]<=stPt[0] or Map.shape[1]<=gPt[0] or gPt[1]<0 or stPt[1]<0:
		print("Start or Goal point is outside the Map")
		return
	if Map[int(stPt[1]),int(stPt[0])] != 255 or Map[int(gPt[1]),int(gPt[0])] != 255:
		print("Start or Goal point is on the obstacle")
		return
	print("Starting A*")
	init = time.time()
	runAStar(Map, stPt, gPt, res, 10.0, anime= anime, weight= w)
	fin = time.time()
	print("Execution Time:",fin-init)
	plt.show()

Parser = argparse.ArgumentParser()
Parser.add_argument('--Start', default= [71,901], \
		help='enter the start point as tuple, Default is (71,901)', nargs='+', type= float)
Parser.add_argument('--Goal',  default= [1021,235],\
		help='enter the goal point as tuple, Default is (1021,235),(452,381)', nargs='+', type= float)
Parser.add_argument('--RoboDia', default= 35.4,\
		help='enter the size of the robot, for a point please enter 0. Default is 10',type=float)
Parser.add_argument('--Weight', default= 1.0,\
		help='Weight value for weighted A*, Default is 1',type=float)
Parser.add_argument('--Anima', default= 'T',\
		help='boolean to show the live animation of path exploration, Default is True')

Args = Parser.parse_args()
stNode = tuple(Args.Start)
gNode = tuple(Args.Goal)
#Time 
res = 10.0
roboDia = Args.RoboDia
anime = booleanParser(Args.Anima)
w = Args.Weight

if __name__ == '__main__':
	run()