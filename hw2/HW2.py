#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example


#### YOUR IMPORTS GO HERE ####
import time
import openravepy
from sympy import *  
from copy import deepcopy 
if not __openravepy_build_doc__:
    from openravepy import *
from numpy import *
#### END OF YOUR IMPORTS ####



def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

#Node for saving state of position
class Node:
    def __init__(self, point3d):
        self.point = point3d;
        self.parent = None
        self.H = 0
        self.G = 0
        self.dif_x = 0
        self.dif_y = 0

    def __eq__(self, other):
        return self.point.x == other.point.x and self.point.y == other.point.y

    def __hash__(self):
        new_hash = hash((self.point.x, self.point.y))
        return new_hash

#Defining content of point
class Point3D:
    def __init__(self, array3d):
        self.x = array3d[0]
        self.y = array3d[1]
        self.z = array3d[2]

    def __repr__(self):
        return "(%s, %s, %s)"%(self.x, self.y, self.z)

    def __len__(self):
        return 0#len(self.x) + len(self.y) +  len(self.z) 

#Check if current node is goal node
def checkNode(point1, point2):
    if (abs(point1.point.x-point2.point.x)<0.01)&(abs(point1.point.y-point2.point.y)<0.01):#&(abs(point1.point.z-point2.point.z)<0.01):
        # point1.parent = point2
        return True
    else:
        return False

#Check for collision
def check_collision(temp_neibhour, env, robot):
    point = [temp_neibhour.point.x,temp_neibhour.point.y,temp_neibhour.point.z]
    robot.SetActiveDOFValues(point)
    return env.CheckCollision(robot)

#Gets the valid neibhour
def getNeibhour(currentNode, env, robot, handles):
    step_size = 0.1
    diagonal_ssize = step_size*1.4
    possibleMoves = [[0,step_size,0],[0,-step_size,0],[step_size,0,0],[-step_size,0,0],[step_size,step_size,0],[-step_size,step_size,0],[step_size,-step_size,0],[-step_size,-step_size,0]]
    valid_neibhour = set()
    #checking collisions with envrinment
    for i in possibleMoves:
        temp_neibhour = Node(Point3D([currentNode.point.x + i[0],currentNode.point.y + i[1],currentNode.point.z + i[2]]))
        if(check_collision(temp_neibhour, env, robot)):
            #print red on in-valid points
            point = [temp_neibhour.point.x,temp_neibhour.point.y,temp_neibhour.point.z]
            handles.append(env.plot3(points=array(point),pointsize=4.0,colors=array(((1,0,0)))))
            
        else:   
            #print blue on valid points
            point = [temp_neibhour.point.x,temp_neibhour.point.y,temp_neibhour.point.z]
            handles.append(env.plot3(points=array(point),pointsize=4.0,colors=array(((0,0,1)))))
            valid_neibhour.add(temp_neibhour)
  
    return valid_neibhour

#calculate h value EUCLEADEAN OR MANHATTEN
def calculate_H(point1,point2):  
    #manhattan
    #return abs(point1.point.x-point2.point.x) + abs(point1.point.y-point2.point.y) #+ abs(point1.point.z-point2.point.z)

    #EUCLEDEAN
    return ((point1.point.x-point2.point.x)**2 + (point1.point.y-point2.point.y)**2 )**0.5

#Check if node exist in closed list or not #NOT USED
def checkclosedSet(point1, closedSet):
    present = 0
    for point2 in closedSet:
        if (abs(point1.point.x-point2.point.x)<0.01)&(abs(point1.point.y-point2.point.y)<0.01):#&(abs(point1.point.z-point2.point.z)<0.01):
            present = 1

    return present

#Check if present node should be included in closed set or not
def checktoPass(point1, closedSet, new_cost):
    present = 1
    for point in closedSet:
        if point == point1:
            if new_cost < point.G:
                present = 1
            else:
                present = 0
    return present

#THE MAIN A*
def astar(start, goal, env, robot):
    with env:
        #Initialisations
        startNode =  Node(Point3D(start))
        goalNode  =  Node(Point3D(goal))

        #set instead of list will increase the efficiency
        openSet   =  set() 
        closedSet =  set()

        #initialise:: G ||| H ||| currentNode ||| addtoopenSet  
        startNode.H =  calculate_H(startNode, goalNode)
        startNode.G =  0

        currentNode =  startNode
        openSet.add(currentNode)

        step_cost = 0.05
        count = 0
        #Running till open loop is non-empty or goal state is not reached
        while(len(openSet)!=0):
            #get current node, minimum value node from opennode
            currentNode = min(openSet, key=lambda gg:gg.H + gg.G)

            #check if current node is goal
            if(checkNode(currentNode, goalNode)):
                # currentNode.parent = goalNode
                break

            #get valid neibhours
            neibhourSet = set()
            neibhourSet = getNeibhour(currentNode, env, robot, handles)

            openSet.remove(currentNode)
            closedSet.add(currentNode)            
            #get closde set from neibhours
            for neighborNode in neibhourSet:
                #calculate the cost and get the min of openset
                
                new_cost = currentNode.G + step_cost
                if(checktoPass(neighborNode, closedSet, new_cost)):
                    neighborNode.G = new_cost
                    neighborNode.H = calculate_H(neighborNode, goalNode)
                    neighborNode.parent = currentNode
                    openSet.add(neighborNode)
                    count = count + 1

        print count
    return currentNode




if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);
    handles =[]

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        # startconfig =[-3.4, -1.4, 0*pi]
        goalconfig = [2.6,-1.3,-pi/2]
        #### YOUR CODE HERE ####
        startconfig = robot.GetActiveDOFValues()
        #### Implement the A* algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.
        
        path = set()
        xy_startconfig =[-3.4, -1.4, .05]
        xy_goalconfig = goalconfig

        startConfig = robot.GetActiveDOFValues()
        # startNode = Node(startConfig
        final = astar(startConfig, goalconfig, env, robot)

        # #### Draw your path in the openrave here (see /usr/lib/python2.7/dist-packages/openravepy/_openravepy_0_8/examples/tutorial_plotting.py for examples)
        # #### Draw the X and Y components of the configurations explored by A*
        final_path = []
        currentNODE = final
        currentNODE.point = [currentNODE.point.x,currentNODE.point.y,-pi/2]#currentNODE.point.z]
        final_path.append(currentNODE.point)
        currentNODE = currentNODE.parent
        prevPoint = []
        print '------------------------------------------------------------------------'
        
        while(True):
            #to get the robot move with proper orientation, atan2(y,x) is given as third value, where, x,y:difference between present and last coordinate
            point_prev = [currentNODE.point.x ,currentNODE.point.y ]
            currentNODE = currentNODE.parent
            point_curr = [currentNODE.point.x ,currentNODE.point.y ] #math.atan2(currentNODE.point.y,currentNODE.point.x)]
            point_final = [point_curr[0]-point_prev[0],point_curr[1]-point_prev[1]]
            point = [currentNODE.point.x,currentNODE.point.y,math.atan2(point_final[1],point_final[0])]
            final_path.append(point)
            point12 = [currentNODE.point.x,currentNODE.point.y,0.05]
            handles.append(env.plot3(points=array(point12),pointsize=4.0,colors=array(((0,0,0)))))
            if xy_startconfig[0] == currentNODE.point.x and xy_startconfig[1] == currentNODE.point.y:
                break
        # #### Now that you have computed a path, execute it on the robot using the controller. You will need to convert it into an openrave trajectory. You can set any reasonable timing for the configurations in the path. Then, execute the trajectory using robot.GetController().SetPath(mypath);
        #plotting trajectory
        traj = RaveCreateTrajectory(env,'')
        traj.Init(robot.GetActiveConfigurationSpecification())     
        for step in final_path:
            traj.Insert(0,(step))         
        planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=1)
        print 'duration',traj.GetDuration()
        print robot.GetActiveDOFValues()
        robot.SetActiveDOFValues(goalconfig)
    robot.GetController().SetPath(traj)
    robot.WaitForController(0)

        #### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

