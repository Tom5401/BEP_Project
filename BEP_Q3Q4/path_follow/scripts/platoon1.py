#!/usr/bin/env python3

import rospy
import numpy as np
import threading
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped


def SteeringController():
    global I_s
    alpha = np.arctan2((Y_pl-Y_ld),(X_pl-X_ld))
    theta = alpha - rot_pl - np.pi
    if theta > np.pi:
        theta = theta - 2*np.pi
    if theta < -np.pi:
        theta = theta + 2*np.pi
    P_s = (theta - ref_theta)*KP_steering
    I_s += (theta - ref_theta)*KI_steering
    steering = P_s + I_s
    if abs(steering) > 1.0:
        steering = steering/abs(steering)
    #info = 'steering:' + str(steering) + ' I:' + str(I_s) + ' P:' + str(P_s)
    #rospy.loginfo(info)
    return steering


def throttleController():
    global sequence, I_t
    dist = np.sqrt((X_pl-X_ld)**2+(Y_pl-Y_ld)**2)
    if abs(X_pl) > 2.8 or abs(Y_pl) > 1.2:
        throttle = 0
    elif sequence == pl_data.header.seq:
        throttle = 0
    else:
        P_t = (dist - ref_dist)*KP_throttle
        I_t += (dist - ref_dist)*KI_throttle
        throttle = P_t + I_t
        info = 'throttle:' + str(throttle) + ' I:' + str(I_t) + ' P:' + str(P_t) + ' dist:' + str(dist)
        rospy.loginfo(info)
        if throttle > 0.3:
            throttle = 0.3
        sequence = pl_data.header.seq
    return throttle


def Publisher():
    while not rospy.is_shutdown():
        if pl_data != None:                  #Test if opti_data has been defined
            steering = SteeringController()
            throttle = throttleController()
            pub_steering.publish(steering)
            pub_throttle.publish(throttle)
        rate.sleep()

def leadCallback(data):
    global ld_data,X_ld,Y_ld,rot_ld
    ld_data = data
    X_ld = ld_data.pose.position.x
    Y_ld = ld_data.pose.position.y
    rot_ld = ld_data.pose.orientation.y


def platoonCallback(data):
    global pl_data,X_pl,Y_pl,rot_pl
    pl_data = data
    X_pl = pl_data.pose.position.x
    Y_pl = pl_data.pose.position.y
    rot_pl = pl_data.pose.orientation.y


def stopJetracer():
    rospy.Rate(5).sleep()
    pub_steering.publish(0.0)
    pub_throttle.publish(0.0)
    rospy.loginfo('Jetracer_2 shutdown')


ref_dist = 0.5
ref_theta = 0
KP_throttle = 0.1
KI_throttle = 0.001
KP_steering = 1.0
KI_steering = 0


pl_data = None
ld_data = None
sequence = 0
I_t = 0.14
I_s = 0

if __name__ == '__main__':
    try:
        rospy.init_node('platoon_car_2', anonymous=False)
        rate = rospy.Rate(10)
        rospy.on_shutdown(stopJetracer)
        pub_steering = rospy.Publisher('steering2',Float32,queue_size=8)
        pub_throttle = rospy.Publisher('throttle2',Float32,queue_size=8)
        rospy.Subscriber('Optitrack_data_topic_2', PoseStamped, leadCallback, queue_size=1,  buff_size=2**24)
        rospy.Subscriber('Optitrack_data_topic_1', PoseStamped, platoonCallback, queue_size=1,  buff_size=2**24)
        t = threading.Thread(target=Publisher)
        t.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

