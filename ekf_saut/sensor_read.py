#!/usr/bin/env python
import rospy
import numpy as np
from medusa_msgs.msg import mUSBLFix
from dsor_msgs.msg import Measurement
from ekf_saut.msg import MsgEKF
from ekf import EKF_simple
import sys
import itertools

#init [range, elevation, bearing]
measures = np.array([-1000, -1000, -1000])
vel = [0, 0]
yaw = -1000
utm_pos = [4290794.43, 491936.56]

measurement_flag = False
gt_pos = [-1, -1]
def callback_beacon (data):
    global measures
    global measurement_flag
    measures = np.array([data.range, data.elevation, data.bearing])
    measurement_flag = True

def callback_vel(data):
    u = data.value[0]
    v = data.value[1]
    global vel
    vel = [u, v]


def callback_yaw(data):
    global yaw
    yaw = data.value[2]


if __name__ == '__main__':
    rospy.init_node('custom_listener', anonymous=True)
    rate = rospy.Rate(10)
    state_dim =2 #alterar para ter o yaw
    measurement_dim = 3
    z = 1.5
    process_mean = 0 
    process_variance = 0.000000001
    measurement_variance = 0.00000000001
    measurement_mean = 0
    #em NED, ENU = (-20, 30)
    x_beacon = 30
    y_beacon = 20
    t_step = 1

    ekf = EKF_simple(state_dim, measurement_dim, process_mean, process_variance, measurement_mean, measurement_variance,z)
    rospy.Subscriber("/mvector/measurement/velocity", Measurement, callback_vel)
    rospy.Subscriber("/mvector/measurement/orientation", Measurement, callback_yaw)
    rospy.Subscriber("/mvector/sensors/usbl_fix", mUSBLFix, callback_beacon)
    pub = rospy.Publisher('chatter', MsgEKF, queue_size=1)
    pose = MsgEKF()
        
    counter = 1
    while not rospy.is_shutdown():
        if measurement_flag:
            ekf.compute_iteration(x_beacon, y_beacon, vel[0], vel[1], yaw, measures, t_step)
            measurement_flag = False
        
            # ENU
            #pose.state.x = ekf.getCurrent_State()[1, 0] #+ utm_pos[0]
            #pose.state.y = ekf.getCurrent_State()[0, 0] #+ utm_pos[1]
            # NED
            pose.state.x = ekf.getCurrent_State()[0, 0]
            pose.state.y = ekf.getCurrent_State()[1, 0]
            pose.state.z = 0
            pose.yaw = yaw
            cov = ekf.getCovariance().tolist()
            pose.covariance = list( itertools.chain.from_iterable( cov ))


            pub.publish(pose)

        rate.sleep()
