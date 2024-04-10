#!/usr/bin/env python3
import rospy
import numpy as np
import threading
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped


#P controller for steering
def SteeringController():
    alpha = np.arctan2((Ycar-point[1]),(Xcar-point[0]))
    theta = alpha - rotation - np.pi
    if theta > np.pi:
        theta = theta - 2*np.pi
    if theta < -np.pi:
        theta = theta + 2*np.pi
    steering = -(ref-theta)*K
    if abs(steering) > 1.0:
        steering = steering/abs(steering)
    info = 'rotation:',str(rotation),'alpha:',str(alpha),'theta:',str(theta),'steering:',str(steering)
    rospy.loginfo(info)
    return steering

#simple throttle controller for when the jetracers leaves the optitrack area
def throttleController():
    global sequence
    if abs(Xcar) > 2.8 or abs(Ycar) > 1.2:
        throttle = 0
    elif sequence == opti_data.header.seq:
        throttle = 0
    else:
        throttle = throttle_gain
        sequence = opti_data.header.seq
    return throttle

#checks if the current waypoint has been reached and defines new waypoint
def nextPoint():
    global point, npoint
    distP = np.sqrt((Xcar-point[0])**2+(Ycar-point[1])**2)
    if distP < point_radius:
        if npoint < (len(points)-1):
            npoint = npoint+1
            point = points[npoint]
        else:
            npoint = 0
            point = points[npoint]
    info = 'distP:',str(distP),'point:',str(point[0]),str(point[1]),str(npoint)
    rospy.loginfo(info)


#Publish to the steering and throttle topics.
def Publisher():
    while not rospy.is_shutdown():
        if opti_data != None:                  #Test if opti_data has been defined
            nextPoint()
            steering = SteeringController()
            throttle = throttleController()
            pub_steering.publish(steering)
            pub_throttle.publish(throttle)
        rate.sleep()


#Assign optitrack data to global variables so they can be used in all functions
def OptiCallback(data):
    global opti_data,Xcar,Ycar,rotation
    opti_data = data
    Xcar = opti_data.pose.position.x
    Ycar = opti_data.pose.position.y
    rotation = opti_data.pose.orientation.y

#set throttle and steering to 0 when the node is shutdown
def stopJetracer():
    rospy.Rate(5).sleep()
    pub_steering.publish(0.0)
    pub_throttle.publish(0.0)
    rospy.loginfo('Jetracer shutdown')


K = 1.0
throttle_gain = 0.15
ref = 0
d = 1.5


a = d/np.sqrt(2)
b = 0.5*(d-a)

'''
points = np.array([[0,0.8],
    [1.5,0],
    [0,-0.8],
    [-1.5,0]])
'''
points = np.array([[0,0],
    [b,a/2],
    [b+a,a/2],
    [b+a,-a/2],
    [b,-a/2],
    [0,0],
    [-b,a/2],
    [-(b+a),a/2],
    [-(b+a),-a/2],
    [-b,-a/2],
    [0,0]])

'''
t = np.linspace(0,2*np.pi,20)
points = np.zeros((len(t),2))

for i in range(len(t)):
    points[i,0] = d*np.sin(t[i])
    points[i,1] = d*np.sin(t[i])*np.cos(t[i])
'''

point = points[0]
npoint = 0
point_radius = 0.3

opti_data = None
sequence = 0


if __name__ == '__main__':
    try:
        rospy.init_node('lead_car', anonymous=False)
        rate = rospy.Rate(10)
        rospy.on_shutdown(stopJetracer)
        pub_steering = rospy.Publisher('steering',Float32,queue_size=8)
        pub_throttle = rospy.Publisher('throttle',Float32,queue_size=8)
        rospy.Subscriber('Optitrack_data_topic_1', PoseStamped, OptiCallback, queue_size=1,  buff_size=2**24)
        t = threading.Thread(target=Publisher)
        t.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




