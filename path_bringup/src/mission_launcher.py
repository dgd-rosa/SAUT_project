#!/usr/bin/env python3
import time
import rospy
import switch_controllers as ctl
import switch_paths as pth

# The entry point of the program
if __name__ == "__main__":
    
    # Get the name of the vehicle being used, controller and path from the ROS parameter server
    vehicle_name = rospy.get_param('vehicle_name', 'myellow')
    path_type = rospy.get_param('path_type', 'bernoulli')
    controller_type = rospy.get_param('controller_type', 'aguiar')
    vehicle_speed = float(rospy.get_param('vehicle_speed_profile', 0.5))

    # Get the UTM coordinates where the inertial frame is located
    utm_north = float(rospy.get_param('utm_north', '4290794.432828876'))
    utm_east = float(rospy.get_param('utm_east', '491936.56077393395'))
    
    # Wait for the required path services to start
    for service in pth.path_services_required:
        rospy.wait_for_service(vehicle_name + service)

    # Wait for the required controller services to start
    for controller_name, service in ctl.controllers_medusa_dic.items():
        rospy.wait_for_service(vehicle_name + service)

    # Reset the current path in memory (start with a clean canvas)
    pth.reset_path(vehicle_name)

    # Spawn a path for the vehicle to follow
    if path_type == 'lawn_mower':
        pth.spawn_lawn_small(center_x=utm_north, center_y=utm_east, z=0.0, vehicle_name=vehicle_name, speed=vehicle_speed)
        #pth.spawn_lawn_mower(center_x=utm_north-40, center_y=utm_east-50, z=0.0, vehicle_name=vehicle_name, speed=vehicle_speed)
    elif path_type == 'bernoulli':
        pth.spawn_bernoulli(radius=7.5, center_x=utm_north, center_y=utm_east, z=0.0, vehicle_name=vehicle_name, speed=vehicle_speed)
    elif path_type == 'simple':
        #pth.path_near_beacon(center_x=utm_north, center_y=utm_east, z=0.0, vehicle_name=vehicle_name, speed=vehicle_speed)
        pth.simple_path(center_x=utm_north, center_y=utm_east, z=0.0, vehicle_name=vehicle_name, speed=vehicle_speed)
    elif path_type == 'big':
        pth.path_big(center_x=utm_north, center_y=utm_east, z=0.0, vehicle_name=vehicle_name, speed=vehicle_speed)
    elif path_type == 'circle':
        pth.path_circle(center_x=utm_north, center_y=utm_east, z=0.0, vehicle_name=vehicle_name, speed=vehicle_speed)
    else:
        rospy.WARN('Desired path only supports lawn_mower or bernoulli in this demo!')
        rospy.signal_shutdown('Desired path not supported')

    # Check if the required controller is in the controllers supported list:
    if controller_type not in ctl.controllers_medusa_dic.keys():
        rospy.WARN('Desired path only supports the following controller_type:')
        rospy.WARN(ctl.controllers_medusa_dic.keys())
        rospy.signal_shutdown('Desired controller not supported')

    # Ask for the desired controller for this experiment
    ctl.spawn_medusa_controller(controller_type, vehicle_name)  