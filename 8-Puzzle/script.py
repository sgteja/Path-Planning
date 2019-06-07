"""
ENPM661 Spring 2019: Project-1 (8 Puzzle)

Author: 
Gnyana Teja Samudrala (sgteja@terpmail.umd.edu)
UID:115824737
University of Maryland, College Park"""

import numpy as np 
from copy import copy
import os

def solve(INIT,goal,max_iter):
	#Initialisation of the variables
	Nodes = np.empty([3,3,1],dtype='int32')
	NodesInfo = np.empty([1,3],dtype='int32')

	Nodes[:,:,0] = INIT
	NodesInfo[0,:] = np.array([1,0,0],dtype='int32')
	cost = 0
	ctr_node = 0
	prt_node = 0
	while True:
		if prt_node>max_iter:
			print("----------Not able to find the solution in given number of iterations----------")
			print("--------Loading data to the text files------------------")
			NodePath = Nodes
			writeFiles(Nodes,NodesInfo,NodePath)
			return np.empty([3,3,1],dtype='int32')
		zero_idx = np.where(Nodes[:,:,prt_node]==0)
		if (zero_idx[0]+1<3):
			
			prt_temp = copy(Nodes[:,:,prt_node])
			temp = move_x(prt_temp,1,zero_idx)
			if checkNodes(copy(Nodes),temp):
				Nodes = np.dstack((Nodes,temp))
				cost = NodesInfo[prt_node,2]+1
				ctr_node+=1
				NodesInfo = np.append(NodesInfo,[[ctr_node,prt_node,cost]],axis=0)
				if checkGoal(Nodes[:,:,ctr_node],goal):
					break
		if zero_idx[0]-1>=0 :
			prt_temp = copy(Nodes[:,:,prt_node])
			temp = move_x(prt_temp,-1,zero_idx)
			if checkNodes(copy(Nodes),temp):
				Nodes = np.dstack((Nodes,temp))
				cost = NodesInfo[prt_node,2]+1
				ctr_node+=1
				NodesInfo = np.append(NodesInfo,[[ctr_node,prt_node,cost]],axis=0)
				if checkGoal(Nodes[:,:,ctr_node],goal):
					break
		if (zero_idx[1]+1<3):
			
			prt_temp = copy(Nodes[:,:,prt_node])
			temp = move_y(prt_temp,1,zero_idx)
			if checkNodes(copy(Nodes),temp):
				Nodes = np.dstack((Nodes,temp))
				cost = NodesInfo[prt_node,2]+1
				ctr_node+=1
				NodesInfo = np.append(NodesInfo,[[ctr_node,prt_node,cost]],axis=0)
				if checkGoal(Nodes[:,:,ctr_node],goal):
					break
		if zero_idx[1]-1>=0 :
			
			prt_temp = copy(Nodes[:,:,prt_node])
			temp = move_y(prt_temp ,-1,zero_idx)
			if checkNodes(copy(Nodes),temp):
				Nodes = np.dstack((Nodes,temp))
				cost = NodesInfo[prt_node,2]+1
				ctr_node+=1
				NodesInfo = np.append(NodesInfo,[[ctr_node,prt_node,cost]],axis=0)
				if checkGoal(Nodes[:,:,ctr_node],goal):
					break
		prt_node+=1
		if(prt_node%1000==0):
			print("Done with -->"+str(prt_node))

	NodePath = np.empty([3,3,1],dtype='int32')
	current = NodesInfo[ctr_node,0]
	parent = NodesInfo[ctr_node,1]
	NodePath = copy(Nodes[:,:,0])
	
	temp_nodes = []
	while True: 
		temp_nodes.append(current)
		if parent==0:
			break
		index = np.where(NodesInfo[:,0]==parent)
		current = NodesInfo[index[0][0],0]
		parent = NodesInfo[index[0][0],1]

	for l in reversed(temp_nodes):
		NodePath = np.dstack((NodePath,Nodes[:,:,l]))

	print("--------Loading data to the text files------------------")
	writeFiles(Nodes,NodesInfo,NodePath)

	return NodePath

def writeFiles(Nodes,NodesInfo,NodePath):
	op_file1 = open('./OutputFiles/Nodes.txt','w+')
	op_file2 = open('./OutputFiles/NodesInfo.txt','w+')
	op_file3 = open('./OutputFiles/NodePath.txt','w+')
	for i in range(Nodes.shape[2]):
		for j in (Nodes[:,:,i].flatten('F')):
			op_file1.write(str(j)+'\t')
		op_file1.write('\n')
	op_file1.close()

	for i in  range(NodesInfo.shape[0]):
		op_file2.write(str(NodesInfo[i,0])+'\t'+str(NodesInfo[i,1])+'\t'+str(NodesInfo[i,2])+'\n')
	op_file2.close()

	for i in range(NodePath.shape[2]):
		for j in (NodePath[:,:,i].flatten('F')):
			op_file3.write(str(j)+'\t')
		op_file3.write('\n')
	op_file3.close()
	return

def move_x(statex,dispx,z_idx):
	statex[z_idx[0],z_idx[1]] = statex[z_idx[0]+dispx,z_idx[1]]
	statex[z_idx[0]+dispx,z_idx[1]] = 0
	return statex

def move_y(statey,dispy,z_idx):
	statey[z_idx[0],z_idx[1]] = statey[z_idx[0],z_idx[1]+dispy]
	statey[z_idx[0],z_idx[1]+dispy] = 0
	return statey

def checkGoal(curr,goal):
	return (curr==goal).all()

def checkNodes(All, current):
	for i in range(All.shape[2]):
		if checkGoal(current.flatten(),All[:,:,i].flatten()):
			return False
	return True

def solvable(initial):
	flat = initial.flatten()
	sol=0
	for i in range(9):
		for j in range(i+1,9,1):
			if flat[i]-flat[j]>0 and flat[i]-flat[j]!=flat[i]:
				sol+=1
				
	if sol%2==0:
		return True
	else:
		return False

def main():

	#Reading the input
	print("-Enter the start configuration separated by comma (default=0,1,3,4,2,5,7,8,6)-")
	input_int = raw_input('-->')
	if input_int == '':
		input_int = "0,1,3,4,2,5,7,8,6"
	input_int = input_int.split(',')
	intial = []
	for i in input_int:
		intial.append(int(i))
	Nodes_Init = np.reshape(intial,(3,3))
	print("----------The start configuration given----------")
	print(Nodes_Init)
	print("----------If correct please enter Yes else enter No(default=Yes)----------")
	status = raw_input('-->')
	if status == '':
		status = "Yes"
	if status=="No":
		main()

	Goal = np.array([[1,2,3],[4,5,6],[7,8,0]],dtype='int32')
	print("----------The goal configuration is----------")
	print(Goal)
	print("----------If correct please enter Yes else enter No(default=Yes)----------")
	status = raw_input('-->')
	if status == '':
		status = "Yes"
	if status=="No":
		input_int = raw_input('-->')
		if input_int == '':
			input_int = "1,2,3,4,5,6,7,8,0"
		input_int = input_int.split(',')
		intial = []
		for i in input_int:
			intial.append(int(i))
		Goal = np.reshape(intial,(3,3))
		print("----------The goal configuration is----------")
		print(Goal)

	print("----------Enter the maximum iterations (default=100000)----------")
	max_iter = raw_input('-->')
	if max_iter == '':
		max_iter = "100000"
	max_iter = int(max_iter)

	print("------If Nodes Path is to be printed enter Yes else enter No(default=Yes)------")
	print_status = raw_input('-->')
	if print_status == '':
		print_status = "Yes"

	if (not(os.path.isdir('./OutputFiles'))):
		os.makedirs('./OutputFiles')

	if solvable(Nodes_Init):
		print("-----------------Solving-----------------------")
		NodesInfo = solve(Nodes_Init,Goal,max_iter)
		if print_status=="Yes":
			for l in range(NodesInfo.shape[2]):
				print(NodesInfo[:,:,l])
	else:
		print("----------Solution doesnot exist for the given configuration----------")

if __name__ == '__main__':
    main()
