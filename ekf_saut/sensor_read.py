#!/usr/bin/env python
import rospy
#import /mnt/nfs/home/dgd_rosa/dsor/catkin_ws_saut/src/medusa_simulation/medusa_base/dsor_utils/dsor_msgs/msg/Measurement.msg
from dsor_msgs.msg import Measurement

def callback (data):
    print(data.value)
    print("\n")

if __name__ == '__main__':
    print("ola")
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("/mvector/measurement/orientation", Measurement, callback)
    rospy.spin()