#!/usr/bin/env python3
import rospy
import numpy as np
import threading
from scipy import spatial
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped


#Optimal steering controller
def steeringController():
    global i_prev,I_angle
    Cx = length*np.cos(rotation) + Xcar
    Cy = length*np.sin(rotation) + Ycar


    if i_prev-interval < 0:
        path_seg = np.concatenate((path[i_prev-interval:], path[:i_prev+interval]))
    else:
        path_seg = path[i_prev-interval: i_prev+interval]

    d_abs,i = spatial.KDTree(path_seg).query([Cx,Cy])
    i += i_prev - interval
    #print('i,i_prev:',i,i_prev)
    i_prev = i


    theta_e = rotation - theta_d[i]
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
    print('segment index:',i_prev-interval,i_prev+interval,i)
    print('theta_e:',str(theta_e),'d_e:',str(d_e),'p_theta:',str(P_angle),'p_d:',str(P_dist),'I_theta',str(I_angle))
    return steering_input 


#simple throttle controller for when the jetracers leaves the optitrack area
def throttleController():
    global sequence
    if abs(Xcar) > 2.8 or abs(Ycar) > 1.3:
        throttle = 0
    elif sequence == opti_data.header.seq:
        throttle = 0
    else:
        throttle = throttle_gain
        sequence = opti_data.header.seq
    return throttle

#Publish to the steering and throttle topics.
def Publisher():
    while not rospy.is_shutdown():
        if opti_data != None:                  #Test if opti_data has been defined
            steering = steeringController()
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

length = 0.05
K = 1.0
throttle_gain = 0.14
ref = 0
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


#KP_dist = 3
#KP_angle = 3/2
#KI_angle = 0.005
I_angle = 0


theta_d = []
for i in range(listLength-1):
    point_vector = path[i-1] - path[i+1]
    theta_d.append(np.arctan2(point_vector[1],point_vector[0]))
point_vector = path[listLength-1] - path[0]
theta_d.append(np.arctan2(point_vector[1],point_vector[0]))




opti_data = None
sequence = 0


if __name__ == '__main__':
    try:
        rospy.init_node('lead_car', anonymous=False)
        rate = rospy.Rate(10)
        rospy.on_shutdown(stopJetracer)
        pub_steering = rospy.Publisher('steering1',Float32,queue_size=8)
        pub_throttle = rospy.Publisher('throttle1',Float32,queue_size=8)
        rospy.Subscriber('Optitrack_data_topic_1', PoseStamped, OptiCallback, queue_size=1,  buff_size=2**24)
        rospy.Rate(1).sleep()
        C0x = length*np.cos(rotation) + Xcar
        C0y = length*np.sin(rotation) + Ycar
        d_e_prev,i_prev = spatial.KDTree(path).query([C0x,C0y])
        print('initial index:',i_prev, d_e_prev)
        t = threading.Thread(target=Publisher)
        t.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




