## Contents

* src:
    * client\_test.py: example client to test request\_image service.
    * sony\_cam\_node.py: ROS node to manage the camera, publishes liveview and offers the take picture service.

* launch:
    * pic_liveview_test.launch: launch file to test node functions, starts image_view nodes to preview published images.
    
* misc:
    * available_api_qx1.txt: available api functions in QX1 camera.

## API Reference

###Liveview###
The camera sends repeatedly, jpeg pictures of low quality, these are published in **liveview/compressed** topic, as **CompressedImage** type message.

###Take photo###
To obtain a high quality picture, make a request to service **sony_cam/request_image**, with a service type GetPolledImage, then the image will be published in **hdpicture/compressed** topic, as **CompressedImage** type message. One service request must be made for every picture.

## Tests

To test the liveview and request_image service, use

$ roslaunch sony\_cam pic\_liveview\_test.launch

This starts two image_view nodes to preview the high quality and the liveview images.
