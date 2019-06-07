import numpy as np 
import matplotlib.pyplot as plt 
import math
import pickle

def getNCoord(act,time,point, rad, L):

	theta=point[2]
	for t in range(1,int(time)+1):
		dx = (rad/2.0)*(act[0]+act[1])*t*math.cos(theta)+point[0]
		dy = (rad/2.0)*(act[0]+act[1])*t*math.sin(theta)+point[1]
		theta = ((rad/L)*(act[0]-act[1])*t)+point[2]
	cost = cost2Go((point[0],point[1]),(dx,dy),1)
	return dx,dy,theta,cost

def newPoints(currentPt, cSpace, time, RPM1, RPM2, rad = 38e-1, L = 23.0):
	
	[ymax,xmax] = cSpace.shape
	actSpace = np.array([[0,RPM1],[RPM1,0],[RPM1,RPM1]
		,[0,RPM2],[RPM2,0],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1]])
	newPts = np.empty((0,4),dtype=np.float32)
	acts = np.empty((0,2),dtype=np.float32)
	[px,py,cost,theta] = currentPt

	for act in actSpace:
		skip=False
		ntheta=theta
		for t in range(1,int(time)+1):
			nx = (rad/2.0)*(act[0]+act[1])*t*math.cos(ntheta)+px
			ny = (rad/2.0)*(act[0]+act[1])*t*math.sin(ntheta)+py
			ntheta = ((rad/L)*(act[0]-act[1])*t)+theta
			if nx<0 or nx>=xmax or ny<0 or ny>=ymax:
				skip=True
				break
			if cSpace[int(ny), int(nx)] != 255:
				skip=True
				break
		if skip:
			continue
		currCost = cost2Go((px,py),(nx,ny),1)
		
		newPts = np.append(newPts,[[nx,ny,currCost+cost,ntheta]],axis= 0)
		acts = np.append(acts,[act],axis=0)

	return newPts,acts

def getID(x,y):
	
	x=int(x)
	y=int(y)
	ndigit = max(len(str((x))),len(str((y))))
	IDx = str((x))
	if len(str(x)) < ndigit:
		for f in range(ndigit-len(str(x))):
			IDx = str(0)+IDx
	IDy = str((y))
	if len(str(y)) < ndigit:
		for f in range(ndigit-len(str(y))):
			IDy = str(0)+IDy
	ID = IDx+IDy

	return ID

def ID2pt(ID):

	ndigit = len((ID))/2
	x = float(ID[0:ndigit])
	y = float(ID[ndigit:])

	return (x,y)

def cost2Go(pt1, pt2, weight = 1.0):

	[x1,y1] = pt1
	[x2,y2] = pt2
	dist = math.sqrt(((x1-x2)**2 + (y1-y2)**2))* weight

	return dist

def explore(cSpace, startPt, goalPt, time, thres, anime= True, weight = 1):

	time = float(time)
	[sx,sy] = startPt
	sID = getID(np.float32(sx),np.float32(sy))
	Q = np.array([[sx,sy,0,0]], dtype=np.float32)
	Qind = []
	Qind.append(sID)
	visited = []
	visited.append(sID)
	cost = {}
	cost[sID] = 0
	parent = {}
	parent[sID] = str(-1)
	actions = {}
	actions[sID] = (0,0)
	mapSize = cSpace.shape
	[gx,gy] = goalPt
	goalID = getID(gx,gy)
	ctr = 0
	stat=False
	reachedPt = None

	while Q.shape[0]!=0:
		
		indMinCost = np.nanargmin(Q, axis=0)[2]
		currID = getID(Q[indMinCost,0],Q[indMinCost,1])
		
		if anime:
			ctr+=1
			plt.plot(Q[indMinCost,0],Q[indMinCost,1], 'rx')
			if ctr%10 == 0:
				plt.pause(0.001)

		newNodes,actlist = newPoints(Q[indMinCost,:], cSpace, time, 3.0,2.0)
		Q = np.delete(Q, indMinCost, 0)
		Qind.remove(currID)
		for i,node in enumerate(newNodes):
			tempID = getID(node[0],node[1])
			tempCost = node[2] + cost2Go((node[0],node[1]),(gx,gy), weight)
			if tempID not in visited:
				cost.update({tempID:node[2]})
				parent.update({tempID: currID})
				tempQ = [[node[0],node[1],tempCost,node[3]]]
				Q = np.append(Q,tempQ,axis=0)
				actions.update({tempID: (actlist[i,0],actlist[i,1])})
				Qind.append(tempID)
				visited.append(tempID)
				if cost2Go((gx,gy),(node[0],node[1]),1)-thres < 0.0:
					reachedPt = (node[0],node[1])
					stat = True
					break
			else:
				if cost[tempID]>node[2]:
					cost.update({tempID:node[2]})
					parent.update({tempID: currID})
					actions.update({tempID: (actlist[i,0],actlist[i,1])})
				if tempID in Qind:
					ind = Qind.index(tempID)
					Q[ind,2] = cost[tempID]+cost2Go((Q[ind,0],Q[ind,1]),(gx,gy), weight)
		
		if stat:
			break
	return parent,actions,reachedPt

def pathPts(tree, goalPt, acts):

	print('Making the path')
	[gx,gy] = goalPt
	goalID = getID(gx,gy)
	pathX = []
	pathY = []
	finalVel = []
	rosList = []
	
	while True:
		pathPts = ID2pt(goalID)
		finalVel.append(acts[goalID])
		pathX.append(pathPts[0])
		pathY.append(pathPts[1])
		rosList.append((acts[goalID][0],acts[goalID][1],pathPts[0],pathPts[1]))
		goalID = tree.get(goalID)
		if goalID == str(-1):
			break
		

	return pathX, pathY, finalVel,rosList


def runAStar(cSpace, startNode, goalNode, time, thres, anime= True, weight= 1.0):

	[sx,sy] = startNode
	[gx,gy] = goalNode
	[sizy,sizx] = cSpace.shape

	# Code for visualization
	fig, ax = plt.subplots()
	fig.suptitle('Astar', fontsize= 16)
	plt.imshow(cSpace, cmap="gray")
	plt.plot(sx,sy,'go')
	plt.plot(gx,gy,'bo')
	ax.set_xlim([0,sizx])
	ax.set_ylim([sizy,0])

	#Run the A* algorithm
	print("Exploring the path....")
	path,acts, gPt = explore(cSpace, (sx,sy), (gx,gy), time, thres, anime= anime, weight= weight)
	print("Getting the points of the path....")
	if gPt == None:
		print "Not possible to move from this configuration"
		return
	px,py,vel, ros = pathPts(path, gPt, acts)
	ros.reverse()
	ros.remove((0,0,sx,sy))
	simSX,simSY = (sx-555)*1e-2,(505-sy)*1e-2
	print("Astar Completed!!")
	print("Set the start point in kobuki launch file as:{}".format((simSX,simSY)))
	rosDict = {'start':[sx,sy],'vel':ros}

	with open('path.pkl','wb') as f:
		pickle.dump(rosDict,f)

	#Plotting the path
	plt.plot(px,py)
	plt.draw()
	plt.pause(0.1)


	return