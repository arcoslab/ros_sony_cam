#!/usr/bin/env python

"""
Test node to test the request_image service from sony_cam_node,
asks for a photo repeatedly

ROS Node - Sony Camera
Copyright (C) 2018  Alexander Marin

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
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
