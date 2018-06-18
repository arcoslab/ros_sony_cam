#!/usr/bin/env python

"""
Main node for Sony Camera

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
import time
import urllib2
from genpy.rostime import Time
from sensor_msgs.msg import CompressedImage
from polled_camera.srv import GetPolledImage, GetPolledImageResponse
from pysony import SonyAPI, ControlPoint, common_header, payload_header
from threading import Thread


##########################################################################
# Constants to control waiting time for specific actions

TIMEOUT_TAKEPIC = 8  # time limit for take-hd-picture service
TIMEOUT_FINDCAM = 5  # time limit for finding camera
TIMEOUT_GETLIVEVIEW = 5  # time limit for getting one liveview frame


##########################################################################
# Related to camera use
class CameraHandler:

    def __init__(self):
        self.find_camera()
        self.hdpic_seq = 0
        self.hdpicture = None
        self.hdpic_tstamp = Time(0, 0)
        self.hdpic_msg = CompressedImage()  # message to publish
        self.cameras = None
        self.cam = None
        self.mode = None
        self.hdpic_resp = None
        self.polled_image_resp = None
        self.hdpic_pub = None
        self.incoming = None
        self.img_msg = CompressedImage()
        self.pub = None
        self.success_liveview = False

    def find_camera(self, duration=3):
        """
        Uses the discover protocol and sets communication with one camera.
        """
        # get available camera
        search = ControlPoint()
        self.cameras = search.discover(duration)

        # use first found camera
        if len(self.cameras):
            self.cam = SonyAPI(self.cameras[0])
        else:
            self.cam = None
            rospy.logerr("No camera found")
            return
            
        self.mode = self.cam.getAvailableApiList()

        # some cameras need startRecMode call
        # no needed in QX1
        if 'starRecMode' in (self.mode['result'])[0]:
            self.cam.startRecMode()
            time.sleep(5)
        else:
            rospy.loginfo("No startRecMode available")

        # re-read capabilities    
        self.mode = self.cam.getAvailableApiList()

        rospy.loginfo("Found camera")
        return
    
    def polled_image_error(self, msg):
        """
        Returns a failed GetPolledImageResponse.
        :param msg: Error message to return with GetPolledImageResponse.
        :return: GetPolledImageResponse.
        """
        rospy.logerr(msg)
        return GetPolledImageResponse(
                    success=False,
                    status_message=msg, 
                    stamp=Time(0))

    def take_pic_thread(self):
        """
        Takes one hd picture and saves it to polled_image_resp.
        :return: None
        """
        self.polled_image_resp = None
        try:
            self.cam.setShootMode(param=['still'])

            # set timestamp for picture
            now = time.time()
            self.hdpic_tstamp = Time(now)
            
            # get status snapshot of cam
            event = self.cam.getEvent(param=[False])

            if "error" in event:
                self.polled_image_resp = self.polled_image_error(str(event['error']))
                return

            # check if is available to take pic
            if event['result'][1]['cameraStatus'] != 'IDLE':
                rospy.loginfo("Camera is busy")
                self.polled_image_resp = self.polled_image_error("Camera is busy")
                return
            
            # take pic
            self.hdpic_resp = self.cam.actTakePicture()
            if 'error' in self.hdpic_resp:
                rospy.logerr(self.hdpic_resp['error'])
                self.polled_image_resp = self.polled_image_error(str(self.hdpic_resp['error']))
                return

            # download pic    
            url = self.hdpic_resp['result'][0][0].replace('\\', '')
            self.hdpicture = urllib2.urlopen(url).read()
            self.hdpic_seq += 1  # increment sequence counter

            rospy.loginfo("Picture taken")
            
            # publish one pic
            self.pub_hdpic()
            # service response
            self.polled_image_resp = GetPolledImageResponse(
                    success=True,
                    status_message="Picture taken",
                    stamp=self.hdpic_tstamp)
            return

        except Exception as err:
            rospy.logerr(str(err))
            self.polled_image_resp = self.polled_image_error("Couldn't take picture")
            return

    def take_picture(self, req):
        """
        Callback function to handle polled camera requests.
        :return: None
        """
        # starts take pic thread and waits TIMEOUT_TAKEPIC seconds to complete
        t_takepic = Thread(target=self.take_pic_thread)
        t_takepic.start()
        t_takepic.join(TIMEOUT_TAKEPIC)

        # checks if take picture action was succesf
        if self.polled_image_resp:
            return self.polled_image_resp
        else:
            return self.polled_image_error("Couldn't take picture, it took too long")

    def set_serv_pic(self):
        """
        Sets the service to take pictures.
        :return: None
        """
        # service setup
        s = rospy.Service('sony_cam/request_image', GetPolledImage, self.take_picture)
        rospy.loginfo("Ready to take pictures")
        
        # waits for requests        
        rospy.spin()
    
    def prep_pub_hdpic(self):
        """
        Set up of hd picture publisher.
        """
        self.hdpic_pub = rospy.Publisher('hdpicture/compressed', CompressedImage, queue_size=1)
        
    def pub_hdpic(self):
        """
        Publishes one hdpicture message.
        """    
        # fill message fields
        self.hdpic_msg.header.seq = self.hdpic_seq
        self.hdpic_msg.header.stamp = self.hdpic_tstamp
        self.hdpic_msg.header.frame_id = "right_sony_cam"
        
        self.hdpic_msg.format = 'jpeg'
        self.hdpic_msg.data = self.hdpicture
        # end fill
         
        self.hdpic_pub.publish(self.hdpic_msg)

    def liveview_thread(self):
        """
        Publishes one liveview frame.
        :return: None
        """
        try:
            # read next image
            self.success_liveview = False
            data = self.incoming.read(8)
            common = common_header(data)
            data = self.incoming.read(128)

            self.img_msg = CompressedImage()  # message to publish
            if common['payload_type'] == 1:  # jpeg frame
                payload = payload_header(data)
                image_file = self.incoming.read(payload['jpeg_data_size'])

                # fill message fields
                self.img_msg.header.seq = common['sequence_number']
                self.img_msg.header.stamp.secs = common['time_stamp'] / 1000.
                self.img_msg.header.stamp.nsecs = common['time_stamp'] * 1000.
                self.img_msg.header.frame_id = "right_sony_cam"

                self.img_msg.format = 'jpeg'
                self.img_msg.data = image_file
                # end fill

                self.pub.publish(self.img_msg)

            self.success_liveview = True

        except Exception as err:
            rospy.logerr("Couldn't get liveview")

    def reset_liveview(self):
        """
        Executes necessary steps to begin a liveview.
        :return: Boolean, True when there are no errors.
        """
        # disables packets with liveview frame info(face detection frames, focus frames and tracking frames)
        # supported in QX1
        try:
            if 'setLiveviewFrameInfo' in (self.mode['result'])[0]:
                self.cam.setLiveviewFrameInfo([{"frameInfo": False}])

            self.incoming = urllib2.urlopen(self.cam.liveview())

        except Exception:
            rospy.logerr("Couldn't reset liveview")
            return False

        else:
            return True

    def liveview(self):
        """
        Begins liveview streaming.
        """
        # publisher setup
        self.pub = rospy.Publisher('liveview/compressed', CompressedImage, queue_size=10)
        rospy.loginfo("Beginning liveview")

        reset_f = True
        while reset_f:
            try:
                reset_f = not self.reset_liveview()
            except Exception:
                rospy.logerr("Couldn't begin liveview set up")
                time.sleep(3)

        while not rospy.is_shutdown():
            try:
                t_getlive = Thread(target=self.liveview_thread)
                t_getlive.start()
                t_getlive.join(TIMEOUT_GETLIVEVIEW)

                if not self.success_liveview:
                    rospy.logerr("Couldn't get liveview, it took too long or other errors")
                    t_getresetlive = Thread(target=self.reset_liveview)
                    t_getresetlive.start()
                    t_getresetlive.join(TIMEOUT_GETLIVEVIEW)
                    time.sleep(3)

            except Exception as err:
                rospy.logerr("Couldn't get liveview")
                time.sleep(3)


def main():

    rospy.init_node('sony_camera')
    
    cam_hand = CameraHandler()

    while not cam_hand.cam:
        try:
            cam_hand.find_camera(TIMEOUT_FINDCAM)
        except Exception:
            pass

    # start liveview publisher
    thread_liveview = Thread(target=cam_hand.liveview)
    thread_liveview.start()
    
    # start image_request service
    thread_serv_pics = Thread(target=cam_hand.set_serv_pic)
    thread_serv_pics.start()
    
    # start hd image publisher
    cam_hand.prep_pub_hdpic()
    
    while not rospy.is_shutdown():
        pass
        
            
if __name__ == '__main__':
    main()
