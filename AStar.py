import numpy as np 
import matplotlib.pyplot as plt 


def newPoints(currentPt, cSpace, res):
	
	[ymax,xmax] = cSpace.shape
	incSid = np.array([[0,1],[1,0],[0,-1],[-1,0]])
	incDiag = np.array([[1,1],[1,-1],[-1,-1],[-1,1]])
	newPts = np.empty((0,3),dtype=np.float32)
	[px,py,cost] = currentPt

	for step in incSid:
		[nx,ny] = [px,py] + step
		if nx>=0 and nx*res<xmax and ny>=0 and ny*res<ymax:
			if cSpace[int(ny*res), int(nx*res)] == 255:
				newPts = np.append(newPts,[[nx,ny,1+cost]],axis= 0)

	for step in incDiag:
		[nx,ny] = [px,py] + step
		if nx>=0 and nx*res<xmax and ny>=0 and ny*res<ymax:
			if cSpace[int(ny*res), int(nx*res)] == 255:
				newPts = np.append(newPts,[[nx,ny,2**0.5+cost]],axis= 0)

	return newPts

def getID(x,y):
	
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
	dist = (((x1-x2)**2 + (y1-y2)**2)**0.5)* weight

	return dist

def explore(cSpace, startPt, goalPt, gridSiz, anime= True, weight = 1):

	
	gridSiz = float(gridSiz)
	[sx,sy] = startPt
	[sx,sy] = [(sx/gridSiz), (sy/gridSiz)]
	sID = getID(sx,sy)
	Q = np.array([[sx,sy,0]], dtype=np.float32)
	Qind = []
	Qind.append(sID)
	visited = []
	visited.append(sID)
	cost = {}
	cost[sID] = 0
	parent = {}
	parent[sID] = str(-1)
	mapSize = cSpace.shape
	[gx,gy] = goalPt
	[gx,gy] = [(gx/gridSiz), (gy/gridSiz)]
	goalID = getID(gx,gy)
	ctr = 0

	while Q.shape[0]!=0:

		indMinCost = np.nanargmin(Q, axis=0)[2]
		currID = getID(Q[indMinCost,0],Q[indMinCost,1])
		
		if anime:
			ctr+=1
			plt.plot(Q[indMinCost,0]*gridSiz,Q[indMinCost,1]*gridSiz, 'rx')
			if ctr%1000 == 0:
				plt.pause(0.001)


		if goalID in parent:
			break 

		newNodes = newPoints(Q[indMinCost,:], cSpace, gridSiz)
		Q = np.delete(Q, indMinCost, 0)
		Qind.remove(currID)
		for node in newNodes:
			tempID = getID(node[0],node[1])
			if tempID not in visited:
				tempCost = node[2] + cost2Go((node[0],node[1]),(gx,gy), weight)
				cost.update({tempID:node[2]})
				parent.update({tempID: currID})
				tempQ = [[node[0],node[1],tempCost]]
				Q = np.append(Q,tempQ,axis=0)
				Qind.append(tempID)
				visited.append(tempID)
			else:
				if cost[tempID]>node[2]:
					cost.update({tempID:node[2]})
					parent.update({tempID: currID})
				if tempID in Qind:
					Q[Qind.index(tempID),2] = node[2]+cost2Go((node[0],node[1]),(gx,gy), weight)
					
	return parent

def pathPts(tree, goalPt, res):

	print('Making the path')
	[gx,gy] = goalPt
	res = float(res)
	[gx,gy] = [(gx/res), (gy/res)]
	goalID = getID(gx,gy)
	pathX = []
	pathY = []
	
	while True:
		pathPts = ID2pt(goalID)
		pathX.append(pathPts[0]*res)
		pathY.append(pathPts[1]*res)
		goalID = tree.get(goalID)
		if goalID == str(-1):
			break
		

	return pathX, pathY

def runAStar(cSpace, startNode, goalNode, gridSiz, anime= True, weight= 1.0):

	[sx,sy] = startNode
	[gx,gy] = goalNode
	[sizy,sizx] = cSpace.shape

	# Code for visualization
	fig, ax = plt.subplots()
	fig.suptitle('Astar', fontsize= 16)
	axXPt = sizx/gridSiz
	axYPt = sizy/gridSiz
	axXPts = []
	axYPts = []

	for i in range(int(gridSiz+1)):
		axXPts.append(((i)*(axXPt)))
		axYPts.append(((i)*(axYPt)))

	ax.set_xticks(axXPts, minor= True)
	ax.set_yticks(axYPts, minor= True)
	ax.xaxis.grid(True, which='minor')
	ax.yaxis.grid(True, which='minor')

	plt.imshow(cSpace, cmap="gray")
	plt.plot(sx,sy,'go')
	plt.plot(gx,gy,'bo')
	ax.set_xlim([0,sizx])
	ax.set_ylim([sizy,0])

	#Run the dijkstra algorithm
	print("Exploring the path....")
	path = explore(cSpace, (sx,sy), (gx,gy), gridSiz, anime= anime, weight= weight)
	print("Getting the points of the path....")
	px,py = pathPts(path, (gx,gy), gridSiz)
	print("Astar Completed!!")

	#Plotting the path
	plt.plot(px,py)
	plt.draw()
	plt.pause(0.1)

	return