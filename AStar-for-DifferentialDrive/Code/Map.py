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

def makeMap(diaDisc, clearance, visual= False):
	"""From the map given we can calculate the corners of the square
		the reference axis of the points is taken considering the top left corner
		as the origin and y-axis pointing downwards"""

	#Creating the empty map
	obstMap = np.zeros((1010,1110),dtype= np.int32)
	m = obstMap.copy()
	totalSiz = diaDisc+(2*clearance)

	
	#Drawing the square and enlarging
	# enlm1 = enlPolyObs(m1.copy(), sqrPts, totalSiz)

	rectPts1 = [(832,0),(832,183),(918,183),(918,0)]
	rectPts2 = [(983,0),(983,91),(1026,91),(1026,0)]
	rectPts3 = [(744,313),(744,389),(1110-1,389),(1110-1,313)]
	rectPts4 = [(1052,444.5),(1052,561.5),(1110-1,561.5),(1110-1,444.5)]
	rectPts5 = [(1019,561.5),(1019,647.5),(1110-1,647.5),(1110-1,561.5)]
	rectPts6 = [(1052,714.75),(1052,831.75),(1110-1,831.75),(1110-1,714.75)]
	rectPts7 = [(927,899),(927,975),(1110-1,975),(1110-1,899)]
	rectPts8 = [(685,975),(685,1010-1),(1110-1,1010-1),(1110-1,975)]
	rectPts9 = [(779,917),(779,975),(896,975),(896,917)]
	rectPts10 = [(474,823),(474,975),(748,975),(748,823)]
	rectPts11 = [(529,669),(529,745),(712,745),(712,669)]
	rectPts12 = [(438,512),(438,695),(529,695),(529,512)]
	rectPts13 =  [(784,626),(784,743),(936,743),(936,626)]
	rectPts_1_1 = [(149.95,100),(149.95,259.9),(309.73,259.9),(309.73,100)]

	cir_1 = [(390,45),81.0/2.0]
	cir_2 = [(438,274),81.0/2.0]
	cir_3 = [(438,736),81.0/2.0]
	cir_4 = [(390,965),81.0/2.0]
	cir_5 = [(149.95,179.95),159.9/2.0]
	cir_6 = [(309.73,179.95),159.9/2.0]

	rect_l = [rectPts1,rectPts2,rectPts3,rectPts4,rectPts5,rectPts6,rectPts7,rectPts8,\
	rectPts9,rectPts10,rectPts11,rectPts12,rectPts13,rectPts_1_1]
	cir_l = [cir_1,cir_2,cir_3,cir_4,cir_5,cir_6]
	
	mrli = []
	enlmrli = []
	for i in rect_l:
		mr = convPolygon(m.copy(),i, vis= visual, img_name= 'sqr')
		mrli.append(mr)
		enlmr = enlPolyObs(mr.copy(), i, totalSiz)
		enlmrli.append(enlmr)

	print('rect')
	mcli = []
	enlmcli = []
	for c in cir_l:
		mc = semiAlg(m.copy(),c[0],c[1])
		mcli.append(mc)
		enlmc = semiAlg(m.copy(),c[0],c[1]+(totalSiz/2.0),val=-255)
		enlmcli.append(enlmc)
	print('circle')
	# #Enlarging the border
	mapBorPts = [(0,0),(0,1010-1),(1110-1,1010-1),(1110-1,0)]
	enlm5 = enlPolyObs(np.ones((1010,1110))*255,mapBorPts, totalSiz)
	print('border')

	Map = np.ones((1010,1110))*255
	Map[enlm5==-255] = 125
	for rect in enlmrli:
		Map[rect==-255] = 125
	for cir in enlmcli:
		Map[cir==-255] = 125
	for rect in mrli:
		Map[rect==-100] = 0
	for cir in mcli:
		Map[cir==-100] = 0
	np.save('Map.npy',Map)
	plt.imshow(Map,cmap='gray')
	cv2.imwrite('map.jpg',Map)
	plt.show()
	return obstMap


'''Parser = argparse.ArgumentParser()
Parser.add_argument('--DiaDisc', default=10, \
		help='diameter of the disc (default is set to 10)', type=float)
Parser.add_argument('--clearance', default=0,\
		help='clearance to be considered (default is 0)', type=float)
Parser.add_argument('--vis', default=False,\
		help='Boolean to visualize the intermediate steps of polygon formation(default is False)')
Args = Parser.parse_args()
diaDisc = Args.DiaDisc
clearance = Args.clearance
visual = Args.vis'''

if __name__ == '__main__':
	makeMap(35.4, 10)