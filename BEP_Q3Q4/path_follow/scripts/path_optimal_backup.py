#!/usr/bin/env python3
import rospy
import numpy as np
import threading
from scipy import spatial
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

# Calculate the location of the middle point of the steering axis
# of the jetracer in reference to the middle point
def C_p():
    Cx = length*np.cos(rotation) + Xcar
    Cy = length*np.sin(rotation) + Ycar
    C = [Cx,Cy]
    return C

#Slice the path segment
def slicer(a, lower, upper):
    if lower < 0:
        return np.concatenate((a[lower:], a[:upper]))
    else:
        return a[lower: upper]

#Optimal steering controller
def steeringController():
    global i_prev
    C = C_p()
    path_seg = slicer(path,i_prev-20,i_prev+20)
    d_abs,i = spatial.KDTree(path_seg).query([C[0],C[1]])
    i += i_prev - 20
    #print('i,i_prev:',i,i_prev)
    i_prev = i
    if i < len(u)-1:
        point_vector = path[i-1] - path[i+1]
    else:
        point_vector = path[i-1] - path[0]
    theta_d = np.arctan2(point_vector[1],point_vector[0])
    theta_e = rotation - theta_d
    if theta_e > np.pi:
        theta_e = theta_e - 2*np.pi
    if theta_e < -np.pi:
        theta_e = theta_e + 2*np.pi
    d_e = (C[1]-path[i][1])*np.cos(theta_d) - (C[0]-path[i][0])*np.sin(theta_d)
    steering_input = -(K1*d_e + K2*theta_e)
    if abs(steering_input) > 1.0:
        steering_input = steering_input/abs(steering_input)
    print('segment index:',i_prev-5,i_prev+5,i)
    print('theta_e:',str(theta_e),'d_e:',str(d_e),'p_theta:',str(theta_e*K2),'p_d:',str(d_e*K1))
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
throttle_gain = 0.13
ref = 0
d = 1.5

u = np.linspace(0,2*np.pi,200)
#path_x = d*np.sin(u)
#path_y = d*np.cos(u)*np.sin(u)
path_x = 2.0*np.cos(u)
path_y = 0.8*np.sin(u)
path = np.array([path_x,path_y]).transpose()
print(path)
K1 = 4          #Distance gain
K2 = 2         #Angle gain



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
        rospy.Rate(1).sleep()
        C = C_p()
        d_e_prev,i_prev = spatial.KDTree(path).query([C[0],C[1]])
        print('initial index:',i_prev, d_e_prev)
        t = threading.Thread(target=Publisher)
        t.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




