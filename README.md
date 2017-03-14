## Synopsis

ROS package for Sony cameras using Sony Remote API.

Includes the sony_cam_node.py which provides camera's output, and some scripts to test this node.

## Motivation

The Sony camera QX1 provides images with very good quality, so making a ROS node to use this camera(and all the other cameras supported by the Sony Remote API) will be useful for object recognition and face recognition purposes from a robot.

## Installation

### ROS
A working installation of ROS must be installed, before building and running this package.

Install ROS

http://wiki.ros.org/indigo/Installation/Ubuntu

Then set up the ROS environment

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

### Install pysony

To install pysony module use

$ git clone git@github.com:Bloodevil/sony_camera_api.git

$ cd sony_camera_api/

$ sudo python setup.py install

Note: Current release (0.1.11) will not work with this program.

### Download and build sony_cam package

$ git clone git@github.com:arcoslab/ros_sony_cam.git

$ cd ros_sony_cam/

$ catkin_make

then source setup.bash

$ source devel/setup.bash

this last step must be executed from every terminal before using this package

## Run
Now from the same terminal where we source the setup.bash

We may run the launch file to see the camera's output, liveview (low quality images) and high quality photos.

$ roslaunch sony_cam pic_liveview_test.launch

We may also use rosrun

$ roscore

$ rosrun sony_cam sony_cam_node.py

and see the published messages with 

$ rostopic echo /liveview/compressed


## API Reference: sony_cam_node.py

###Liveview###
The camera sends repeatedly, jpeg pictures of low quality, these are published in **liveview/compressed** topic, as **CompressedImage** type message.

###Take photo###
To obtain a high quality picture, make a request to service **sony_cam/request_image**, with a service type GetPolledImage, then the image will be published in **hdpicture/compressed** topic, as **CompressedImage** type message. One service request must be made for every picture.

## Examples

See client_test.py to learn how to use the service image request.

## Author 
Alexander Marin Drobinoga alexanderm2230@gmail.com

