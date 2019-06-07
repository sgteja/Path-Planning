import numpy as np 
import matplotlib.pyplot as plt
import argparse
import cv2
import sys

def convPolygon(wSpace, pts, val= -100, vis= False, img_name= 'img'):
	""" This function creates the line equation for the points given in the 
		pts list and then gets the region of the polygon using the half planes
		pts must be in counterclockwise direction"""

	noPts = len(pts)
	lines = []
	for i,pt1 in enumerate(pts):
		if noPts == i+1:
			[x2,y2] = pts[0]
		else:
			[x2,y2] = pts[i+1]
		[x1,y1] = pt1
		if x2>x1 :
			sign = -1
		else:
			sign = 1 
		if x2-x1 == 0:
			if y1>y2:
				sign = -1
			lines.append((sign*-1,0,sign*x2))
		else:
			m = float(y2-y1)/float(x2-x1)
			c = y1-(m*x1)
			lines.append((sign*m,sign*-1,sign*c))

	[ylim,xlim] = wSpace.shape 
	[y,x] = np.mgrid[0:ylim,0:xlim]
	negPts = []
	for line in lines:
		temp = line[0]*x+line[1]*y+line[2]
		[a,b] = np.where(temp<=0)
		indices = np.hstack((a.reshape(-1,1),b.reshape(-1,1)))
		negPts.append(indices)

	if vis:
		fig = plt.figure(figsize=(20,5))
		fig.subplots_adjust(wspace=0.25, hspace=0.25)
		for i in range(len(lines)):
			ll = np.zeros_like(wSpace)
			for k in range(negPts[i].shape[0]):
				ll[negPts[i][k,0],negPts[i][k,1]] = 255

			#plt.subplot(1,len(lines),i+1)
			#plt.imshow(ll,cmap='gray')
			ax = fig.add_subplot(2, len(lines), i+1)
			ax.imshow(ll,cmap="gray")
		#plt.show()
		fig.savefig(img_name+'_1')
		sys.stdout.flush()

	finalPts = negPts[0]
	if vis:
		fig = plt.figure(figsize=(20,5))
		fig.subplots_adjust(wspace=0.25, hspace=0.25)
	for i in range(len(negPts)-1):
		set1 = set((tuple(x,) for x in finalPts))
		set2 = set((tuple(x) for x in negPts[i+1]))
		finalPts = np.array([x for x in set2.intersection(set1)])
		if vis:
			ll = np.zeros_like(wSpace)
			for k in range(finalPts.shape[0]):
				ll[finalPts[k,0],finalPts[k,1]] = 255
			ax = fig.add_subplot(2, len(negPts)-1, i+1)
			ax.imshow(ll,cmap="gray")
			#plt.subplot(1,len(lines),i+1)
			#plt.imshow(ll,cmap = 'gray')
	if vis:
		fig.savefig(img_name+'_2')
		sys.stdout.flush()

	for i in range(finalPts.shape[0]):
		wSpace[finalPts[i,0],finalPts[i,1]] = val

	return wSpace

def nonConv(wSpace, pts, val= -100, vis= False, img_name= 'img'):
	"""This function takes in list of convex shapes with there points list for each shape
		and the union of these shapes will give us the desired nonConvex shape"""

	convPts = []
	ctr=0
	for conv in pts:
		ctr+=1
		wSpace1 = convPolygon(wSpace, conv, vis=vis, img_name= img_name+'_'+str(ctr))
		[a,b] = np.where(wSpace1<0)
		indices = np.hstack((a.reshape(-1,1),b.reshape(-1,1)))
		convPts.append(indices)

	finalPts = convPts[0]
	for i in range(len(convPts)-1):
		set1 = set((tuple(x,) for x in finalPts))
		set2 = set((tuple(x,) for x in convPts[i+1]))
		finalPts = np.array([x for x in set2.union(set1)])

	for i in range(finalPts.shape[0]):
		wSpace[finalPts[i,0],finalPts[i,1]] = val

	return wSpace

def semiAlg(wSpace, centPt, majorRad, shape= 'circle', minorRad=0, align= 'h', val=-100):
	""" This function uses the semi algebra concept and plots the shape specified
		shape variable takes in the name of the shape to be plotted and if its a 
		ellipse then we need to specify align as string whether 'h' or 'v' for
		horizontal and vertical major axis respectively."""

	[xc,yc] = centPt
	[ylim,xlim] = wSpace.shape 
	[y,x] = np.mgrid[0:ylim,0:xlim]
	xc = float(xc)
	yc = float(yc)
	majorRad = float(majorRad)
	minorRad = float(minorRad)
	if shape == 'circle':
		temp = (x-xc)**2 + (y-yc)**2 - (majorRad)**2
	else:
		if align == 'h':
			temp = (((x-xc)**2)/majorRad**2) + (((y-yc)**2)/minorRad**2) - 1
		else:
			temp = (((x-xc)**2)/minorRad**2) + (((y-yc)**2)/majorRad**2) - 1

	ind = np.where(temp<=0)
	wSpace[ind] = val

	return wSpace

def enlPolyObs(wSpace, pts, siz, val= -255):
	"""This takes in the corner points of any polygon finds it edges and performs
		Minikowski sum along the edge to enlarge the obstacles, the 'siz' variable
		takes in the diameter of the disc"""

	[ylim,xlim] = wSpace.shape 
	wm = wSpace.copy()
	[fy,fx] = np.mgrid[0:ylim,0:xlim]
	[aFreePts,bFreePts] = np.where(wm>=0)
	indicesFreePts = np.hstack((aFreePts.reshape(-1,1),bFreePts.reshape(-1,1)))
	setFreePts = set((tuple(x) for x in indicesFreePts))
	enlPts= [] 
	for i,pt1 in enumerate(pts):
		if len(pts) == i+1:
			[x2,y2] = pts[0]
		else:
			[x2,y2] = pts[i+1]
		[x1,y1] = pt1
		[xc,yc] = [(x1+x2)/2.0,(y1+y2)/2.0]
		if x2-x1 == 0:
			templ1 = -1*fx+x2 
		else:
			m = float(y2-y1)/(x2-x1)
			c = y1-((m)*x1)
			mx = np.array(fx)*np.float32([((m))])
			templ1 = mx-fy+c
			templ1 = np.int32(templ1)
		dist = ((xc-x2)**2+(yc-y2)**2)
		tempc = (fx-xc)**2 + (fy-yc)**2 - dist
		[al1,bl1] = np.where(templ1==0)
		indicesl1 = np.hstack((al1.reshape(-1,1),bl1.reshape(-1,1)))
		[ac,bc] = np.where(tempc<=0)
		indicesc = np.hstack((ac.reshape(-1,1),bc.reshape(-1,1)))
		set1 = set((tuple(x) for x in indicesl1))
		set2 = set((tuple(x) for x in indicesc))
		edgePt = np.array([x for x in set2.intersection(set1)])
	
		for point in edgePt:
			tempCEdg = (fx-point[1])**2 + (fy-point[0])**2 - (siz/2.0)**2
			[aCPts,bCPts] = np.where(tempCEdg<=0)
			indCPts = np.hstack((aCPts.reshape(-1,1),bCPts.reshape(-1,1)))
			setCPts = set((tuple(x) for x in indCPts))
			setEnlPt = set((x for x in setCPts.intersection(setFreePts)))
			enlPts.append(setEnlPt)
	
	for i,s1 in enumerate(enlPts):
		if len(enlPts) == i+1:
			break
		s2 = enlPts[i+1]
		EnlPts = np.array([x for x in s1.union(s2)])
	
		for f in EnlPts:
			wm[f[0],f[1]] = val

	return wm

def main():
	"""From the map given we can calculate the corners of the square
		the reference axis of the points is taken considering the top left corner
		as the origin and y-axis pointing downwards"""

	#Creating the empty map
	obstMap = np.zeros((150,250),dtype= np.int32)
	m = obstMap.copy()
	totalSiz = diaDisc+(2*clearance)

	
	#Drawing the square and enlarging
	sqrPts = [(100,82.5),(100,37.5),(50,37.5),(50,82.5)]
	m1 = convPolygon(m.copy(),sqrPts, vis= visual, img_name= 'sqr')
	enlm1 = enlPolyObs(m1.copy(), sqrPts, totalSiz)

	#Drawing the hexagon and enlarging
	hexPts = [(173,150-15),(193,150-52),(170,150-90),(163,150-52),(125,150-56),\
			(150,150-15)]
	convPts1 = [hexPts[0],hexPts[1],hexPts[2],hexPts[3]]
	convPts2 = [hexPts[3],hexPts[4],hexPts[5],hexPts[0]]
	m2 = nonConv(m1.copy(),[convPts1,convPts2], vis= visual, img_name= 'hex')
	enlm2 = enlPolyObs(enlm1.copy(), hexPts, totalSiz)

	#Drawing the ellipse and enlarging
	m3 = semiAlg(m2.copy(), (140,150-120),30/2,'ellipse',12/2)
	enlm3 = semiAlg(m2.copy(), (140,150-120),(30/2)+totalSiz/2,\
		'ellipse',(12/2)+totalSiz/2,'h',-225)

	#Drawing the circle and enlarging
	m4 = semiAlg(m3.copy(), (190,150-130),30/2)
	enlm4 = semiAlg(enlm3.copy(), (190,150-130),(30/2)+totalSiz/2,val= -225)

	#Enlarging the border
	mapBorPts = [(0,0),(0,149),(249,149),(249,0)]
	enlm5 = enlPolyObs(enlm2.copy(),mapBorPts, totalSiz)


	obstMap[m4>=0] = 255
	if visual:
		fig = plt.figure(figsize=(20,5))
		fig.subplots_adjust(wspace=0.25, hspace= 0.25)
		ax = fig.add_subplot(1,1,1)
		ax.imshow(obstMap, cmap= 'gray')
		fig.savefig('ObstMap')
	obstMap[enlm4==-225] = 125
	obstMap[enlm5==-255] = 125
	obstMap[m4==-100] = 0
	np.save('Map.npy', obstMap)
	fig = plt.figure(figsize=(20,5))
	fig.subplots_adjust(wspace=0.25, hspace= 0.25)
	ax = fig.add_subplot(1,1,1)
	ax.imshow(obstMap, cmap= 'gray')
	fig.savefig('map')
	#cv2.imwrite('map.jpg',obstMap)

Parser = argparse.ArgumentParser()
Parser.add_argument('--DiaDisc', default=10, \
		help='diameter of the disc (default is set to 10)', type=float)
Parser.add_argument('--clearance', default=0,\
		help='clearance to be considered (default is 0)', type=float)
Parser.add_argument('--vis', default=False,\
		help='Boolean to visualize the intermediate steps of polygon formation(default is False)')
Args = Parser.parse_args()
diaDisc = Args.DiaDisc
clearance = Args.clearance
visual = Args.vis

if __name__ == '__main__':
	main()