#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time, numpy
import openravepy


if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        #robot.SetActiveDOFValues([1.27843491,-1.7624945652,-0.69799996,1.27843491,-2.32100002,-0.69799996]);
 	robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
               
	robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    #env.Load('robots/puma.robot.xml')
    time.sleep(0.1)
	

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    

    #### YOUR CODE HERE ####
    table5 = [[1,0,0,0.15839],[0,1,0,-1.1869],[0,0,1,0.7400],[0,0,0,1]]

    a = 0.3
    b = 0.55
    handles=[]
    with env:
        count = 0
        tableid = [1,2,3,4,5,6]
        
        for body in env.GetBodies():
            count = count + 1
            print body, count
            if count-1 in tableid:
                print body
                temp = body.GetTransform()
                #print temp
                
                temp_coordinates = numpy.asarray(temp[[0,1,2],[3]], dtype=float)
                #format(temp_coordinates, '.2f')
                #print temp_coordinates
                x = temp_coordinates[0]
                y = temp_coordinates[1]
                z = temp_coordinates[2]
            # handles.append(env.drawlinestrip(points=array(
            #                                  linewidth=50.0,
            #                                  colors=array(1,0,0))))
                if temp[0][0] == 1:
                    handles.append(env.drawlinestrip(points=array(((x + 0.3, y + 0.6, z), (x + 0.3, y - 0.6, z),
                                                                   (x - 0.3, y - 0.6, z), (x - 0.3, y + 0.6, z),
                                                                   (x + 0.3, y + 0.6, z))),
                                                     linewidth=4.0,
                                                     colors=array((1, 0, 0))))
                else:
                    handles.append(env.drawlinestrip(points=array(((x + 0.6, y + 0.3, z), (x + 0.6, y - 0.3, z),
                                                                   (x - 0.6, y - 0.3, z), (x - 0.6, y + 0.3, z),
                                                                   (x + 0.6, y + 0.3, z))),
                                                     linewidth=4.0,
                                                     colors=array((1, 0, 0))))
            # if count == 6:
            #     print body.Get  
            #     body.SetTransform(numpy.array(table5))

            # if count == 7:
            
            #     body.SetTransform(numpy.array(table6))
            radius = math.sqrt(25)
            X = numpy.linspace(-radius, radius, 18)
            for x in X:
                y = math.sqrt((radius * radius) - (x * x))
                handles.append(env.plot3(points=array(((x, y, 0), (x, -y, 0))),
                                         pointsize=5.0,
                                         colors=array((0, 0, 1))))

        #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

