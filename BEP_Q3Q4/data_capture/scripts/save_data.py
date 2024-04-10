#!/usr/bin/env python3
import rospy
import numpy as np
import message_filters
import pandas as pd
import os
import matplotlib.pyplot as plt
from datetime import datetime
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import PoseStamped



def optitrackDataSave(msg,now,car_id):
    time = float(str(msg.header.stamp.secs) + '.' + str(msg.header.stamp.nsecs)) - timezero.to_sec()
    data = [[now.to_sec(),time,msg.header.seq,msg.pose.position.x,msg.pose.position.y,msg.pose.orientation.y]]
    csvfile = directory + 'optitrack' + str(car_id) + 'Data.csv'
    df = pd.DataFrame(data,columns=['time','time_opti','sequence','x','y','rotation'])
    df.to_csv(csvfile,mode='a',header=False,index=False)
    info = str(now.to_sec()) + ':Opti: ' + str(time)
    rospy.loginfo(info)


def optitrack1Callback(msg):
    now = rospy.Time.now() - timezero
    optitrackDataSave(msg,now,1)


def optitrack2Callback(msg):
    now = rospy.Time.now() - timezero
    optitrackDataSave(msg,now,2)


def optitrack3Callback(msg):
    now = rospy.Time.now() - timezero
    optitrackDataSave(msg,now,3)



def throttleDataSave(msg,now,car_id):
    data = [[now.to_sec(),msg.data]]
    csvfile = directory + 'throttle' + str(car_id) + 'Data.csv'
    df = pd.DataFrame(data,columns=['time','throttle'])
    df.to_csv(csvfile,mode='a',header=False,index=False)
    #info = str(now.to_sec()) + ':throttle: ' + str(msg)
    #rospy.loginfo(info)


def throttle1Callback(msg):
    now = rospy.Time.now() - timezero
    throttleDataSave(msg,now,1)

def throttle2Callback(msg):
    now = rospy.Time.now() - timezero
    throttleDataSave(msg,now,2)

def throttle3Callback(msg):
    now = rospy.Time.now() - timezero
    throttleDataSave(msg,now,3)

def steeringDataSave(msg,now,car_id):
    data = [[now.to_sec(),msg.data]]
    csvfile = directory + 'steering' + str(car_id) + 'Data.csv'
    df = pd.DataFrame(data,columns=['time','steering'])
    df.to_csv(csvfile,mode='a',header=False,index=False)
    #info = str(now.to_sec()) + ':steering: ' + str(msg)
    #rospy.loginfo(info)

def steering1Callback(msg):
    now = rospy.Time.now() - timezero
    steeringDataSave(msg,now,1)

def steering2Callback(msg):
    now = rospy.Time.now() - timezero
    steeringDataSave(msg,now,2)

def steering3Callback(msg):
    now = rospy.Time.now() - timezero
    steeringDataSave(msg,now,3)


def plotData():
    fig = plt.figure()
    ax = fig.add_subplot(111)

    u = np.linspace(0,2*np.pi,200)
    #path_x = 1.5*np.cos(u)
    #path_y = 0.9*np.sin(u)

    path_x = 2*np.sin(u)
    path_y = (1/1.6)*np.sin(2*u)

    plt.plot(path_x,path_y)


    df_track1 = pd.read_csv(directory+'optitrack1Data.csv')
    t = df_track1['time'].to_numpy()
    track1_x = df_track1['x'].to_numpy()
    track1_y = df_track1['y'].to_numpy()
    plt.plot(track1_x,track1_y)

    df_track2 = pd.read_csv(directory+'optitrack2Data.csv')
    t = df_track2['time'].to_numpy()
    track2_x = df_track2['x'].to_numpy()
    track2_y = df_track2['y'].to_numpy()
    plt.plot(track2_x,track2_y)

    df_track3 = pd.read_csv(directory+'optitrack3Data.csv')
    t = df_track3['time'].to_numpy()
    track3_x = df_track3['x'].to_numpy()
    track3_y = df_track3['y'].to_numpy()
    plt.plot(track3_x,track3_y)
    
    plt.xlim(-2.2,2.2)
    plt.ylim(-1.2,1.2)
    ax.set_aspect('equal')
    plt.savefig(directory+'/data.png')


if __name__ == '__main__':
    try:
        rospy.init_node('capture', anonymous=False)
        rospy.on_shutdown(plotData)
        timezero = rospy.Time.now()
        timenow = datetime.now()
        timestamp = str(timenow.month) + '-' + str(timenow.day) + '_' + str(timenow.hour) + '-' + str(timenow.minute) + '-' + str(timenow.second)

        #directory = '/home/tombuntu/measurements/run_' + timestamp + '/'
        directory = '/home/tombuntu/measurements/run/'
        os.makedirs(directory, exist_ok=True)
        df_steering = pd.DataFrame(columns=['time','steering'])
        df_throttle = pd.DataFrame(columns=['time','throttle'])
        df_optitrack = pd.DataFrame(columns=['time','time_opti','sequence','x','y','rotation'])
        

        csv_optitrack1 = directory + 'optitrack1Data.csv'
        csv_steering1 = directory + 'steering1Data.csv'
        csv_throttle1 = directory + 'throttle1Data.csv'
        df_optitrack.to_csv(csv_optitrack1,index=False)
        df_steering.to_csv(csv_steering1,index=False)
        df_throttle.to_csv(csv_throttle1,index=False)
        rospy.Subscriber('Optitrack_data_topic_1', PoseStamped, optitrack1Callback,queue_size=10)
        rospy.Subscriber('steering1', Float32, steering1Callback)
        rospy.Subscriber('throttle1', Float32, throttle1Callback)

        csv_steering2 = directory + 'steering2Data.csv'
        csv_throttle2 = directory + 'throttle2Data.csv'
        csv_optitrack2 = directory + 'optitrack2Data.csv'
        df_optitrack.to_csv(csv_optitrack2,index=False)
        df_steering.to_csv(csv_steering2,index=False)
        df_throttle.to_csv(csv_throttle2,index=False)
        rospy.Subscriber('Optitrack_data_topic_2', PoseStamped, optitrack2Callback,queue_size=10)
        rospy.Subscriber('steering2', Float32, steering2Callback)
        rospy.Subscriber('throttle2', Float32, throttle2Callback)

        csv_steering3 = directory + 'steering3Data.csv'
        csv_throttle3 = directory + 'throttle3Data.csv'
        csv_optitrack3 = directory + 'optitrack3Data.csv'
        df_optitrack.to_csv(csv_optitrack3,index=False)
        df_steering.to_csv(csv_steering3,index=False)
        df_throttle.to_csv(csv_throttle3,index=False)
        rospy.Subscriber('Optitrack_data_topic_3', PoseStamped, optitrack3Callback,queue_size=10)
        rospy.Subscriber('steering3', Float32, steering3Callback)
        rospy.Subscriber('throttle3', Float32, throttle3Callback)



        #rospy.Subscriber('Optitrack_tests_3', PoseStamped, optitrack3Callback,queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




