#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from math import pi
from Queue import *
from control_msgs.msg import *
from trajectory_msgs.msg import *

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q0 = [pi/3,-pi/2,0,-pi/4,pi/2,pi/4]
Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
Q3 = [1.5,-0.2,-1.57,0,0,0]

#tilt for maze
I0  = [ 0,  -pi,  pi/2,  0,   pi/2,  0] #initialize

#fa = pi/15 #forward_angle styrofoam
#ba = pi/180 #back_angle
fa = pi/18 #forward_angle wood
ba = pi/40 #back_angle


Xp0 = [ 0,  -pi,  pi/2,  fa,   pi/2,  0]
Xpt = [ 0,  -pi,  pi/2,  ba,   pi/2,  0]

Xn0 = [ 0,  -pi,  pi/2,  -fa,  pi/2,  0]
Xnt = [ 0,  -pi,  pi/2,  -ba,  pi/2,  0]


Yp0 = [ 0,  -pi,  pi/2,  0,  (pi/2 - fa),   0]
Ypt = [ 0,  -pi,  pi/2,  0,  (pi/2 - ba),   0]

Yn0 = [ 0,  -pi,  pi/2,  0,  (pi/2 + fa),  0]
Ynt = [ 0,  -pi,  pi/2,  0,  (pi/2 + ba),  0]


client = None

def move1():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    #g.trajectory.points = [
    #    JointTrajectoryPoint(positions=Q0, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
    #    JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
    #    JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    g.trajectory.points = [
        JointTrajectoryPoint(positions=I0, velocities=[0]*6, time_from_start=rospy.Duration(1.0)),
        JointTrajectoryPoint(positions=Xp0, velocities=[0]*6, time_from_start=rospy.Duration(1.5)),
        JointTrajectoryPoint(positions=Xpt, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=I0, velocities=[0]*6, time_from_start=rospy.Duration(3.5))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def initSet():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    
    g.trajectory.points = [
        JointTrajectoryPoint(positions=I0, velocities=[0]*6, time_from_start=rospy.Duration(5.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

class motion:
    def __init__(self, tilt_time, direction):
        self.t = tilt_time
        self.d = direction


def move_maze(cur_motion): #cur_motion is an object of class motion
    print 'move maze ..........'
    print cur_motion.d
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    ttime = 0.3
    #fr = 0.8 #forward_time_ratio styrofoam
   # br = 0.08 #back_time_ratio
    #ir = 1 - fr - br #initial_time ratio

    fr = 0.6 #forward_time_ratio wood
    br = 0.3 #back_time_ratio
    ir = 1 - fr - br #initial_time ratio

    g.trajectory.points.append(
        JointTrajectoryPoint(positions=I0 , velocities=[0]*6, time_from_start=rospy.Duration(ttime)))
    
    ttime += cur_motion.t*fr

    if cur_motion.d == 1:  #right
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Xp0, velocities=[0]*6, time_from_start=rospy.Duration(ttime)))
            
        ttime += cur_motion.t*br       
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Xpt, velocities=[0]*6, time_from_start=rospy.Duration(ttime)))
    
    elif cur_motion.d == 8: #up
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Yp0, velocities=[0]*6, time_from_start=rospy.Duration(ttime)))
            
        ttime += cur_motion.t*br       
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Ypt, velocities=[0]*6, time_from_start=rospy.Duration(ttime)))
    
    elif cur_motion.d == 4:  #left 
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Xn0, velocities=[0]*6, time_from_start=rospy.Duration(ttime)))
            
        ttime += cur_motion.t*br      
        g.trajectory.points.append(
           JointTrajectoryPoint(positions=Xnt, velocities=[0]*6, time_from_start=rospy.Duration(ttime)))
   

    elif cur_motion.d == 2: #down
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Yn0, velocities=[0]*6, time_from_start=rospy.Duration(ttime)))
            
        ttime += cur_motion.t*br      
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Ynt, velocities=[0]*6, time_from_start=rospy.Duration(ttime)))
    

    #ttime += cur_motion.t*ir    
    #g.trajectory.points.append(
    #    JointTrajectoryPoint(positions=I0, velocities=[0]*6, time_from_start=rospy.Duration(ttime))) 

    #print g.trajectory.points
    client.send_goal(g)
    try:
        print 'try'
        client.wait_for_result()
    except KeyboardInterrupt:
        print 'except'
        client.cancel_goal()
        raise


def move_disordered():
    order = [4, 2, 3, 1, 5, 0]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = [JOINT_NAMES[i] for i in order]
    q1 = [Q1[i] for i in order]
    q2 = [Q2[i] for i in order]
    q3 = [Q3[i] for i in order]
    g.trajectory.points = [
        JointTrajectoryPoint(positions=q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    client.wait_for_result()
    
def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    
    d = 2.0
    g.trajectory.points = []
    for i in range(10):
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    
    client.send_goal(g)
    time.sleep(2.0)
    print "Interrupting"
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def executeStep(direction, tilt_time):
    global client
    try:
        #rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        initSet()
        m = motion(tilt_time, direction)
        print direction
        move_maze(m)


    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__': 
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        #initSet()
        move1()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

