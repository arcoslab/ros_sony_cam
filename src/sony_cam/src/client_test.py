#!/usr/bin/env python

"""
Program to test the request_image service from sony_cam_node
asks for a photo repeatedly
"""

import rospy
from polled_camera.srv import GetPolledImage, GetPolledImageRequest
import time

def take_pics():

    rospy.init_node('picture_client')
    
    rospy.loginfo("Waiting for service")
    rospy.wait_for_service('sony_cam/request_image')
    
    try:
        # send request while 
        while not rospy.is_shutdown():
            take_pic = rospy.ServiceProxy('sony_cam/request_image',GetPolledImage)
            resp = take_pic(GetPolledImageRequest())
            rospy.loginfo("Made request: "+resp.status_message)
            time.sleep(2)
            
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    take_pics()
