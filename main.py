import numpy as np 
import matplotlib.pyplot as plt
import argparse
import warnings
warnings.filterwarnings("ignore")


from Map import makeMap
from Dijkstra import runDijkstra
from AStar import runAStar

def booleanParser(val):
	if val.lower() == 't':
		return True
	else:
		return False

def checkRes(sPts,gPts,res):

	checS = False
	checG = False
	if sPts[0] % res == 0 and sPts[1] % res == 0:
		checS = True
	if gPts[0] % res == 0 and gPts[1] % res == 0:
		checG = True

	return checS,checG

def run():

	stPt = (stNode[0],150-stNode[1])
	gPt = (gNode[0],150-gNode[1])

	cS,cG = checkRes(stPt,gPt,res)
	
	if cS and cG:

		if All:
			print("Making the map for point robot")
			Map = makeMap(0,0)
			
			if Map.shape[1]<=stPt[0] or Map.shape[1]<=gPt[0] or gPt[1]<0 or stPt[1]<0:
				print("Start or Goal point is outside the Map")
				return

			if Map[int(stPt[1]),int(stPt[0])] != 255 or Map[int(gPt[1]),int(gPt[0])] != 255:
				print("Start or Goal point is on the obstacle")
				return

			print("Starting Dijkstra")
			runDijkstra(Map, stPt, gPt, res, anime= anime)

			print("Starting A*")
			runAStar(Map, stPt, gPt, res, anime= anime, weight= w)

			print("Making the map for the rigid body so as to consider it as point")
			Map = makeMap(roboDia, clear)

			if Map.shape[1]<=stPt[0] or Map.shape[1]<=gPt[0] or gPt[1]<0 or stPt[1]<0:
				print("Start or Goal point is outside the Map")
				return
			if Map[int(stPt[1]),int(stPt[0])] != 255 or Map[int(gPt[1]),int(gPt[0])] != 255:
				print("Start or Goal point is on the obstacle")
				return

			print("Starting Dijkstra")
			runDijkstra(Map, stPt, gPt, res, anime= anime)

			print("Starting A*")
			runAStar(Map, stPt, gPt, res, anime= anime, weight= w)

		else:

			print("Making the map so that the rigid body is considered as point")
			Map = makeMap(roboDia, clear)
			plt.imshow(Map)
			plt.plot(stPt[0],stPt[1],'bo')
			plt.plot(gPt[0],gPt[1],'ro')
			plt.show()
			if Map.shape[1]<=stPt[0] or Map.shape[1]<=gPt[0] or gPt[1]<0 or stPt[1]<0:
				print("Start or Goal point is outside the Map")
				return
			if Map[int(stPt[1]),int(stPt[0])] != 255 or Map[int(gPt[1]),int(gPt[0])] != 255:
				print("Start or Goal point is on the obstacle")
				return

			if dijk:
				print("Starting Dijkstra")
				runDijkstra(Map, stPt, gPt, res, anime= anime)

			if AStar:
				print("Starting A*")
				runAStar(Map, stPt, gPt, res, anime= anime, weight= w)

	else:
		if not cS:
			print("Change the start point or resolution, path cannot be found with this resolution")
		else:
			print("Change the goal point or resolution, path cannot be found with this resolution")

	plt.show()

Parser = argparse.ArgumentParser()
Parser.add_argument('--Start', default= [40,130], \
		help='enter the start point as tuple, Default is (40,20)', nargs='+', type= float)
Parser.add_argument('--Goal',  default= [220,10],\
		help='enter the goal point as tuple, Default is (220,140)', nargs='+', type= float)
Parser.add_argument('--Res', default= 5,\
		help='enter the resolution required, Default is 5.0', type=int)
Parser.add_argument('--RoboDia', default= 10,\
		help='enter the size of the robot, for a point please enter 0. Default is 10',type=float)
Parser.add_argument('--Clearance', default= 0,\
		help='enter the clearance required, Default is 0',type=float)
Parser.add_argument('--AStar', default= 'T',\
		help='boolean to determine whether to run A* or not, Default is True')
Parser.add_argument('--Weight', default= 1.0,\
		help='Weight value for weighted A*, Default is 1',type=float)
Parser.add_argument('--Dijk', default= 'T',\
		help='boolean to determine whether to run Dijkstra or not, Default is True')
Parser.add_argument('--Anima', default= 'T',\
		help='boolean to show the live animation of path exploration, Default is True')
Parser.add_argument('--checkAll', default= 'T',\
		help='If True runs all the 4 cases given the robot dia and clearance, Default is True')

Args = Parser.parse_args()
stNode = tuple(Args.Start)
gNode = tuple(Args.Goal)
res = Args.Res
roboDia = Args.RoboDia
clear = Args.Clearance
AStar = booleanParser(Args.AStar)
dijk = booleanParser(Args.Dijk)
anime = booleanParser(Args.Anima)
w = Args.Weight
All = booleanParser(Args.checkAll)


if __name__ == '__main__':
	run()