#!/usr/bin/env python

import rospy
import time
import urllib2
import datetime 
from genpy.rostime import Time
from sensor_msgs.msg import CompressedImage
from polled_camera.srv import GetPolledImage, GetPolledImageResponse
from pysony import SonyAPI, ControlPoint, common_header, payload_header
from threading import Thread

class CameraHandler:

    def __init__(self):
        self.find_camera()
        self.hdpic_seq=0
        self.hdpicture=None
        self.hdpic_tstamp=Time(0,0)
        self.hdpic_msg = CompressedImage() # message to publish
        
    def find_camera(self):
        """
        uses the discover protocol and sets communication with one camera
        """
        # get available camera
        search = ControlPoint()
        self.cameras = search.discover()

        # use first found camera
        if len(self.cameras):
            self.cam = SonyAPI(QX_ADDR=self.cameras[0])
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
        
    def take_picture(self,req):
        """
        callback function to handle polled camera requests
        """
        self.cam.setShootMode(param=['still'])
        
        # set timestamp for picture
        now = time.time()
        self.hdpic_tstamp = Time(now)
        
        # get status snapshot of cam
        event = self.cam.getEvent(param=[False])
        
        if "error" in event:
            rospy.logerr(event['error'])
            return GetPolledImageResponse(success=False,status_message=str(evet['error']), stamp=Time(0))
         
        # check if is available to take pic   
        if (event['result'][1]['cameraStatus']!='IDLE'):
            rospy.loginfo("Camera is busy")
            return GetPolledImageResponse(\
                    success=False,\
                    status_message="Camera is busy",\
                    stamp=Time(0))
        
        # take pic
        self.hdpic_resp = self.cam.actTakePicture()
        if 'error' in self.hdpic_resp:
            rospy.logerr(self.hdpic_resp['error'])
            return GetPolledImageResponse(\
                    success=False,\
                    status_message=str(self.hdpic_resp['error']),\
                    stamp=Time(0))
        
        # download pic    
        url = self.hdpic_resp['result'][0][0].replace('\\','')
        self.hdpicture = urllib2.urlopen(url).read()
        
        self.hdpic_seq += 1 # increment sequence counter
        
        rospy.loginfo("Picture taken")
        
        # publish one pic
        self.pub_hdpic()
        
        # service response
        return GetPolledImageResponse(\
                success=True,\
                status_message="Picture taken",\
                stamp=self.hdpic_tstamp)
    
    def set_serv_pic(self):
        """
        sets the service to take pictures
        """
        # service setup
        s = rospy.Service('sony_cam/request_image', GetPolledImage , self.take_picture)
        rospy.loginfo("Ready to take pictures")
        
        # waits for requests        
        rospy.spin()
    
    def prep_pub_hdpic(self):
        """
        publisher of hd picture set up
        """
        self.hdpic_pub = rospy.Publisher('hdpicture/compressed', CompressedImage, queue_size=1)
        
    def pub_hdpic(self):
        """
        publishes one hdpicture message
        """    
        # fill message fields
        self.hdpic_msg.header.seq = self.hdpic_seq
        self.hdpic_msg.header.stamp = self.hdpic_tstamp
        self.hdpic_msg.header.frame_id = "right_sony_cam"
        
        self.hdpic_msg.format = 'jpeg'
        self.hdpic_msg.data = self.hdpicture
        # end fill
         
        self.hdpic_pub.publish(self.hdpic_msg)
    
    def liveview(self):
        """
        begins liveview streaming
        """
        # publisher setup
        pub = rospy.Publisher('liveview/compressed', CompressedImage, queue_size=10)
        
        # disables packets with liveview frame info(face detection frames, focus frames and tracking frames)
        # supported in QX1
        if 'setLiveviewFrameInfo' in (self.mode['result'])[0]:
            self.cam.setLiveviewFrameInfo([{"frameInfo":False}])    

        incoming = self.cam.liveview()

        rospy.loginfo("Beginning liveview")
        
        img_msg = CompressedImage() # message to publish
        
        while not rospy.is_shutdown():

            # read next image
            data = incoming.read(8)
            common = common_header(data)
            data = incoming.read(128)
            if common['payload_type']==1: # jpeg frame
                payload = payload_header(data)
                image_file = incoming.read(payload['jpeg_data_size'])
                   
                # fill message fields
                img_msg.header.seq = common['sequence_number']
                img_msg.header.stamp.secs = common['time_stamp']/1000.
                img_msg.header.stamp.nsecs = common['time_stamp']*1000.
                img_msg.header.frame_id = "right_sony_cam"
                
                img_msg.format = 'jpeg'
                img_msg.data = image_file
                # end fill
                
                pub.publish(img_msg)
                                      
                # todo: set delay
    
    
def main():

    rospy.init_node('sony_camera')
    
    cam_hand = CameraHandler()
    while (cam_hand.cam==None):
        time.sleep(3)
        cam_hand.find_camera()
    
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
