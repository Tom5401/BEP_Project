#!/usr/bin/env python3
import rospy
import numpy as np
import threading
import os
from scipy import spatial
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped


#Optimal steering controller
def steeringController():
    global i_prev,I_angle
    Cx = length*np.cos(rot_pl) + X_pl
    Cy = length*np.sin(rot_pl) + Y_pl


    if i_prev-interval < 0:
        path_seg = np.concatenate((path[i_prev-interval:], path[:i_prev+interval]))
    else:
        path_seg = path[i_prev-interval: i_prev+interval]

    d_abs,i = spatial.KDTree(path_seg).query([Cx,Cy])
    i += i_prev - interval
    #print('i,i_prev:',i,i_prev)
    i_prev = i


    theta_e = rot_pl - theta_d[i]
    if theta_e > np.pi:
        theta_e = theta_e - 2*np.pi
    if theta_e < -np.pi:
        theta_e = theta_e + 2*np.pi
    d_e = (Cy-path[i][1])*np.cos(theta_d[i]) - (Cx-path[i][0])*np.sin(theta_d[i])

    P_dist = KP_dist*d_e
    P_angle = KP_angle*theta_e
    I_angle += KI_angle*theta_e
    steering_input = -(P_dist + P_angle + I_angle)
    if abs(steering_input) > 1.0:
        steering_input = steering_input/abs(steering_input)
    return steering_input 


#simple throttle controller for when the jetracers leaves the optitrack area
def throttleController():
    global sequence, I_throttle, error_prev
    dist = np.sqrt((X_pl-X_ld)**2+(Y_pl-Y_ld)**2)
    if abs(X_pl) > 2.8 or abs(Y_pl) > 1.2:
        throttle = 0
    elif sequence == pl_data.header.seq:
        throttle = 0
    else:
        error = dist - ref_dist
        P_throttle = error*KP_throttle
        I_throttle += error*0.1*KI_throttle
        D_throttle = (error-error_prev)*KD_throttle/0.1
        error_prev = error
        throttle = P_throttle + I_throttle + D_throttle
        info = 'throttle:' + str(throttle) + ' I:' + str(I_throttle) + ' P:' + str(P_throttle) +'D:'+ str(D_throttle)+ ' dist:' + str(dist)
        print(info)
        if throttle > 0.2:
            throttle = 0.2
        sequence = pl_data.header.seq
    return throttle

#Publish to the steering and throttle topics.
def Publisher():
    while not rospy.is_shutdown():
        if pl_data != None:                  #Test if pl_data has been defined
            steering = steeringController()
            throttle = throttleController()
            pub_steering.publish(steering)
            pub_throttle.publish(throttle)
        rate.sleep()


#Assign optitrack data to global variables so they can be used in all functions
def platoonCallback(data):
    global pl_data,X_pl,Y_pl,rot_pl
    pl_data = data
    X_pl = pl_data.pose.position.x
    Y_pl = pl_data.pose.position.y
    rot_pl = pl_data.pose.orientation.y

def leadCallback(data):
    global ld_data,X_ld,Y_ld,rot_ld
    ld_data = data
    X_ld = ld_data.pose.position.x
    Y_ld = ld_data.pose.position.y
    rot_ld = ld_data.pose.orientation.y

#set throttle and steering to 0 when the node is shutdown
def stopJetracer():
    rospy.Rate(5).sleep()
    pub_steering.publish(0.0)
    pub_throttle.publish(0.0)
    rospy.loginfo('Jetracer shutdown')

length = 0.05
d = 1.5

listLength = 50
interval = int(listLength/20)
u = np.linspace(2*np.pi,0,listLength)
#path_x = 1.5*np.cos(u)
#path_y = 0.9*np.sin(u)

path_x = 2*np.sin(u)
path_y = (1/1.6)*np.sin(2*u)
path = np.array([path_x,path_y]).transpose()
print(path)
KP_dist = 3
KP_angle = 3/2
KI_angle = 0.0
I_angle = 0


KP_throttle = 0.25
KI_throttle = 0.1
KD_throttle = 0.2
I_throttle = 0.11
ref_dist = 0.5
error_prev = 0

gains = ['KP_throttle: '+str(KP_throttle),
'KI_throttle: '+str(KI_throttle),
'KD_throttle: '+str(KD_throttle),
'I_throttle: '+str(I_throttle),
'ref_dist: '+str(ref_dist)+'\n',
'KP_dist: '+str(KP_dist),
'KP_angle: '+str(KP_angle),
'KI_angle: '+str(KI_angle)]


directory = '/home/tombuntu/measurements/run/'
os.makedirs(directory, exist_ok=True)
with open(directory + 'gains_follow_car1.txt','w') as f:
    for line in gains:
        f.write(line)
        f.write('\n')


theta_d = []
for i in range(listLength-1):
    point_vector = path[i-1] - path[i+1]
    theta_d.append(np.arctan2(point_vector[1],point_vector[0]))
point_vector = path[listLength-1] - path[0]
theta_d.append(np.arctan2(point_vector[1],point_vector[0]))


pl_data = None
ld_data = None
sequence = 0


if __name__ == '__main__':
    try:
        rospy.init_node('follow_car1', anonymous=False)
        rate = rospy.Rate(10)
        rospy.on_shutdown(stopJetracer)
        pub_steering = rospy.Publisher('steering2',Float32,queue_size=8)
        pub_throttle = rospy.Publisher('throttle2',Float32,queue_size=8)
        rospy.Subscriber('Optitrack_data_topic_2', PoseStamped, platoonCallback, queue_size=1,  buff_size=2**24)
        rospy.Subscriber('Optitrack_data_topic_1', PoseStamped, leadCallback, queue_size=1,  buff_size=2**24)
        rospy.Rate(1).sleep()
        C0x = length*np.cos(rot_pl) + X_pl
        C0y = length*np.sin(rot_pl) + Y_pl
        d_e_prev,i_prev = spatial.KDTree(path).query([C0x,C0y])
        print('initial index:',i_prev, d_e_prev)
        t = threading.Thread(target=Publisher)
        t.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




