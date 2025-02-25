#!/usr/bin/env python
from math import fabs
from matplotlib.pyplot import flag
import rospy
import numpy as np
from medusa_msgs.msg import mUSBLFix
from dsor_msgs.msg import Measurement
from ekf_saut.msg import MsgEKF
from ekf import EKF_simple, EKF_withOrientation
from auv_msgs.msg import NavigationStatus
import itertools

#init [range, elevation, bearing]
measures = np.array([-1000, -1000, -1000, -1000])
vel = [0, 0]
yaw = -1000
yaw_rate = 0.0
utm_pos = [4290794.43, 491936.56]
nav_pos = [-1000, -1000, -1000]

pose_array = []
nav_array = []
flag_array = []
measurement_flag = False
dead_reck_array = []

comms_fail_time = 80
comms_counter = 0
def callback_beacon (data):
    global measures
    global measurement_flag
    global yaw
    global comms_counter
    measures = np.array([data.range, data.elevation, data.bearing - np.pi, yaw])
    if comms_counter < comms_fail_time:
        comms_counter = comms_counter + 1
        measurement_flag = False
    else:
        measurement_flag = True

def callback_vel(data):
    u = data.value[0]
    v = data.value[1]
    global vel
    vel = [u, v]


def callback_yaw(data):
    global yaw
    global yaw_rate
    yaw = data.value[2]
    yaw_rate = data.value[5]

def callback_nav(data):
    global nav_pos
    nav_pos = []
    nav_pos.append(data.position.north)
    nav_pos.append(data.position.east)
    nav_pos.append(data.orientation.z)

def dead_fcn():
    with open('data_files/comms_ruido.txt', 'w') as f:
        global pose_array
        counter = 0
        for pose in pose_array:
            f.write(str(pose.state.x))
            f.write('\t')
            f.write(str(pose.state.y))
            f.write('\t')
            f.write(str(pose.yaw))
            f.write('\t')
            f.write(str(nav_array[counter][0]))
            f.write('\t')
            f.write(str(nav_array[counter][1]))
            f.write('\t')
            f.write(str(nav_array[counter][2]))
            f.write('\t')
            f.write(str(dead_reck_array[counter][0] + utm_pos[0]))
            f.write('\t')
            f.write(str(dead_reck_array[counter][1]+ utm_pos[1]))
            f.write('\t')
            f.write(str(np.degrees(dead_reck_array[counter][2])))
            f.write('\t')
            f.write(str(pose.covariance))
            f.write('\t')
            f.write(str(flag_array[counter]))
            f.write('\n')
            counter = counter + 1

if __name__ == '__main__':
    rospy.init_node('custom_listener', anonymous=True)
    rate = rospy.Rate(10)
    state_dim =3 #com o yaw
    measurement_dim = 4 #com yaw
    z = 1.5
    process_mean = 0 
    process_variance = 0.01
    measurement_variance = 0.2
    measurement_mean = 0
    
    # x_beacon = 90.0
    # y_beacon = -120.0
    x_beacon = 40.0
    y_beacon = 20.0
    t_step = 0.1

    ekf = EKF_withOrientation(state_dim, measurement_dim, process_mean, process_variance, measurement_mean, measurement_variance, z)
    rospy.Subscriber("/mvector/measurement/velocity", Measurement, callback_vel)
    rospy.Subscriber("/mvector/measurement/orientation", Measurement, callback_yaw)
    rospy.Subscriber("/mvector/sensors/usbl_fix", mUSBLFix, callback_beacon)
    rospy.Subscriber("/mvector/nav/filter/state", NavigationStatus, callback_nav)
    pub = rospy.Publisher('chatter', MsgEKF, queue_size=1)

    while not rospy.is_shutdown():
        ekf.predict(vel[0], vel[1], yaw_rate, yaw, t_step)

        #Update when there is new measurements
        if measurement_flag:
            ekf.update(x_beacon, y_beacon, measures)
            flag_array.append(1)
            measurement_flag = False
        else:
            flag_array.append(0)
        
        print("Current State: " + str(ekf.getCurrent_State()))
        print("********\n")
        print("Covariance: " + str(ekf.getCovariance()))
        print("********\n")

        # NED
        pose = MsgEKF()
        pose.state.x = ekf.getCurrent_State()[0, 0] + utm_pos[0]
        pose.state.y = ekf.getCurrent_State()[1, 0] + utm_pos[1]
        pose.state.z = 0
        pose.yaw = np.degrees(ekf.getCurrent_State()[2,0])
        
        cov = ekf.getCovariance().tolist()
        pose.covariance = list( itertools.chain.from_iterable( cov ))
    
        pose.error = [pose.state.x - nav_pos[0], pose.state.y - nav_pos[1]]
        
        pose_array.append(pose)
        nav_array.append(nav_pos)
        dead_reck = ekf.getDeadReckoning() 
        dead_reck = list(itertools.chain.from_iterable( dead_reck ))
        dead_reck_array.append(dead_reck)

        pub.publish(pose)

        rate.sleep()
    rospy.on_shutdown(dead_fcn)
