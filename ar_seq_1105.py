#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from geometry_msgs.msg import Transform, Vector3, Twist
#from kalman_zumy.srv import ImuSrv,ImuSrvResponse,NuSrv,NuSrvResponse
#import exp_quat_func as eqf
import time
from Step import Step
from test_move2 import *


listener = None
last_trans = [[0]*3]*3
cur_trans = [[0]*3]*3

all_tags = {}

move = ['move up', 'move right', 'hit the goal']
path = []#step
def follow_ar_tag(ar_tags):

    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    print ar_tags
    count = 0

    #print last_trans
    while not rospy.is_shutdown():
        global last_trans, cur_trans
        for i in range(len(ar_tags)):

            try:
                time.sleep(0.15)
                #(cur_trans[i],rot) = listener.lookupTransform('usb_cam', ar_tags[i], rospy.Time(0))

                if(i == len(ar_tags) - 1):
                    print 'hit the goal'
 
                else:
                    (cur_trans[i],rot) = listener.lookupTransform(ar_tags[i], ar_tags[i+1], rospy.Time(0))
                    print ar_tags[i],'----',ar_tags[i+1],"----",cur_trans[i], '------'
                    cur_trans[i] = np.array(cur_trans[i])

                    if(cur_trans[i][0] > 0.01):
                        print "move right"
                    if(cur_trans[i][0] < -0.01):
                        print "move left"
                    if(cur_trans[i][1] > 0.01):
                        print "move up"
                    if(cur_trans[i][1] < -0.01):
                        print "move down"

            #print trans0[0], '---', last_trans[0][0]
                
                '''
                if cur_trans[i][0] == last_trans[i][0]:
                    print move[i] 
                last_trans[i] = cur_trans[i];
                '''
            except:
            #count = count +1
        #print (sys.exc_info())
            #print count
                continue

def isReachable(current_tag, next_artag):
    current = int(current_tag[len(current_tag) - 1]) % 18
    if len(current_tag) == 12 :
        current = int(current_tag[(len(current_tag) - 2):]) % 18
    
    while(True):
        try :
            time.sleep(0.15)
            (trans,rot) = listener.lookupTransform(current_tag, next_artag, rospy.Time(0))
            print current_tag, next_artag, trans
            if trans[0] != 0:
                break
        except:
            print ''
    direction = 0
    require_time = 0.0

    para_time = 7.0 #wood

    if trans[0] > 0.05:# positive x
        direction = direction | 1
        require_time = abs(2*trans[0]*para_time)
    
    if trans[1] < -0.05:# negative y
        direction = direction | 2
        require_time = abs(2*trans[1]*para_time)
    
    if trans[0] < -0.05:# negative x
        direction = direction | 4
        require_time = abs(2*trans[0]*para_time)
    
    if trans[1] > 0.05:# positive y
        direction = direction | 8
        require_time = abs(2*trans[1]*para_time)

    #if current_tag == 'ar_marker_10' :
       # current = 5
    print direction, current, require_time
    res = direction & current
    print res
    d = [1,2,4,8]
    if not (direction in d):
        return (0, 0) 
    if res == 0:
        return (direction, require_time)
    else :
        return (0, 0)

def findNext(current_tag, hasbeen): #assume the amount and the name of ar tag is known
    print 'findNext start.', current_tag
    if current_tag == 'ar_marker_5':
        
        return True
    if current_tag == all_tags[7] or current_tag == all_tags[11] or current_tag == all_tags[13] or current_tag == all_tags[14] :# is 3-walls
        return False
    i = 0
    #for i in range(len(ar_tags)):
    
    up = Step()
    down= Step()
    left = Step()
    right = Step()

    while(i < len(ar_tags)):
        next_artag = ar_tags[i]
        print "i=", i, current_tag, next_artag
        print 'hasbeen'
        isContinue = False
        for tag in hasbeen:
            print tag, next_artag
            if tag == next_artag:
                isContinue = True
                #i += 1
                break
        if current_tag == next_artag:
            i+=1
            continue
        print isReachable(current_tag, next_artag)
        (canReach,req_time) = isReachable(current_tag, next_artag)
        print canReach, req_time
        #if canReach != 0: # if next_artag is along current's axis and there's no wall.
        if canReach == 1 and req_time < right.tilt_time:
            if right.direction_of_ur5 == -1:
                i += 1
                continue
            if next_artag in hasbeen:
                right = Step(direction_of_ur5=-1)
            else:
                right = Step(next_artag, canReach, req_time)
                print "append right"
        
        elif canReach == 2 and req_time < down.tilt_time:
            if down.direction_of_ur5 == -1:
                i += 1
                continue
            if next_artag in hasbeen:
                down = Step(direction_of_ur5=-1)
            else:
                down = Step(next_artag, canReach, req_time)
                print "append down"
        
        elif canReach == 4 and req_time < left.tilt_time:
            if left.direction_of_ur5 == -1:
                i += 1
                continue
            if next_artag in hasbeen:
                left = Step(direction_of_ur5=-1)
            else:
                left = Step(next_artag, canReach, req_time)
                print "append left"
        
        elif canReach == 8 and req_time < up.tilt_time:
            if up.direction_of_ur5 == -1:
                i += 1
                continue
            if next_artag in hasbeen:
                up = Step(direction_of_ur5=-1)
            else:
                up = Step(next_artag, canReach, req_time)
                print "append up"
        
        i += 1

    if up.direction_of_ur5 == 8:
        print '-----------------8--------------------'
        hasbeen.append(up.next_ar)
        b = findNext(up.next_ar, hasbeen)
        if b:
            print "store step: ", up 
            path.append(up)
            return True
        else :
            hasbeen.pop()
    if left.direction_of_ur5 == 4:
        print '-----------------4--------------------'
        hasbeen.append(left.next_ar)
        b = findNext(left.next_ar, hasbeen)
        if b:
            print "store step: ", left 
            path.append(left)
            return True
        else :
            hasbeen.pop()
    if down.direction_of_ur5 == 2:
        print '-----------------2--------------------'
        hasbeen.append(down.next_ar)
        b = findNext(down.next_ar, hasbeen)
        if b:
            print "store step: ", down
            path.append(down)
            return True
        else :
            hasbeen.pop()
    if right.direction_of_ur5 == 1:
        print '-----------------1--------------------'
        hasbeen.append(right.next_ar)
        b = findNext(right.next_ar, hasbeen)
        if b:
            print "store step: ", right 
            path.append(right)
            return True
        else :
            hasbeen.pop()
            
        '''
        if canReach != 0: # if next_artag is along current's axis and there's no wall.
            print '-------------------------------------'
            hasbeen.append(next_artag)
            b = findNext(next_artag, hasbeen)
            if b:
                cur_step = Step(next_artag, canReach, req_time)
                #cur_step.next_ar = next_artag
                #cur_step.direction_of_ur5 = canReach#####
                print "store step: direction is", canReach
                path.append(cur_step) #path starts from the end
                return True
            else :
                hasbeen.pop()
        '''
        #i += 1


def executePath():
    path.reverse()
    print "executing path"
    for step in path:
        print "step:"
        executeStep(step.direction_of_ur5, step.tilt_time)
        count = 0
        while(count < 20):
            try:
                time.sleep(0.15)
                (trans0,rot0) = listener.lookupTransform('usb_cam', step.next_ar, rospy.Time(0))
                trans0 = np.array(trans0)
                #print trans0[0], '---', last_trans[0][0]
                
                if trans0[0] == last_trans[0][0]: #marble blocks the ar tag
                    print 'Arrived'
                    print 'Recovery' 
                    break
                else:
                    count += 1
                if count % 10 == 9 :
                    executeStep(step.direction_of_ur5, step.tilt_time)
                last_trans[0] = trans0;
            except:
                print 'error'
    print 'Done'

"""
        try:
            time.sleep(0.15)
            (trans0,rot0) = listener.lookupTransform('usb_cam', ar_tags['ar0'], rospy.Time(0))
            trans0 = np.array(trans0)
            #print trans0[0], '---', last_trans[0][0]
            if trans0[0] == last_trans[0][0]:
                print 'move_up!' 
            last_trans[0] = trans0;


        except:
            #count = count +1
        #print (sys.exc_info())
            #print count
            continue
        try:
            time.sleep(0.15)

            (trans1,rot1) = listener.lookupTransform('usb_cam', ar_tags['ar1'], rospy.Time(0))
            trans1 = np.array(trans1)
            if trans1[0] == last_trans[1][0]:
                print 'move_right!' 
            last_trans[1] = trans1;
        except:
            #count = count +1
        #print (sys.exc_info())
            #print count
            continue
        try:
            time.sleep(0.15)

            (trans2,rot2) = listener.lookupTransform('usb_cam', ar_tags['ar2'], rospy.Time(0))
            trans2 = np.array(trans2)
            if trans2[0] == last_trans[2][0]:
                print 'hit the goal!' 
            last_trans[2] = trans2;
        except:
            #count = count +1#!/usr/bin/env python

"""

if __name__=='__main__':
    rospy.init_node('follow_ar_tag_twist')
    if len(sys.argv) < 3:
        print('Use: ar_seq_1105.py [ AR tag number for start] [ AR tag number for goal] ')
        sys.exit()
    ar_tags = {}

    for i in range(16):
        all_tags[i] = 'ar_marker_%d'%(i)
    ar_tags[0] = 'ar_marker_' + sys.argv[1]
    ar_tags[1] = 'ar_marker_' + sys.argv[2]
    
    ar_tags[2] = 'ar_marker_' + '3'
    ar_tags[3] = 'ar_marker_' + '1'
    #ar_tags[3] = 'ar_marker_' + '10'
    ar_tags[4] = 'ar_marker_' + '2'
    ar_tags[5] = 'ar_marker_' + '9'
    
    ar_tags[6] = 'ar_marker_' + '12'
    ar_tags[7] = 'ar_marker_' + '6'
    ar_tags[8] = 'ar_marker_' + '31'
    ar_tags[9] = 'ar_marker_' + '13'

    
    #ar_tags[2] = 'ar_marker_' + sys.argv[3]
    
    '''
    ar_tags[2] = 'ar_marker_' + '3'
    ar_tags[3] = 'ar_marker_' + '27'
    ar_tags[4] = 'ar_marker_' + '2'
    ar_tags[5] = 'ar_marker_' + '9'
    
    ar_tags[6] = 'ar_marker_' + '14'
    ar_tags[7] = 'ar_marker_' + '12'
    ar_tags[8] = 'ar_marker_' + '30'
    ar_tags[9] = 'ar_marker_' + '57'
    ar_tags[10] = 'ar_marker_' + '21'
    ar_tags[11] = 'ar_marker_' + '48'
    ar_tags[12] = 'ar_marker_' + '39'
    #ar_tags[13] = 'ar_marker_' + '11'
    ar_tags[13] = 'ar_marker_' + '6'
    '''

    #follow_ar_tag(ar_tags=ar_tags)
    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    hasbeen = [ar_tags[0]]
    findNext(ar_tags[0], hasbeen)
    str = raw_input("Ready?")
    if str == 'start':
        executePath()

    rospy.spin()
