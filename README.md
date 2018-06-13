# ROS Node for Sony Cameras
[TOC]
## Synopsis

ROS package for Sony cameras using Sony Remote API.

Includes the sony_cam_node.py which provides camera's output, and some scripts to test this node.


## Motivation

The Sony camera QX1 provides images with very good quality, so making a ROS node to use this camera(and all the other cameras supported by the Sony Remote API) will be useful for object recognition and face recognition purposes for robotic applications.

## Installation

### ROS

A working installation of ROS must be installed, before building and running this package.

Install ROS, see:
http://wiki.ros.org/indigo/Installation/Ubuntu

Then set up the ROS environment, see:
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

You may also need to install:

- `ros-<distro>-image-transport-plugins`
- `ros-<distro>-image-view`

to visualize images.


### Setup pysony

To setup pysony module, get the pysony repo with:

```bash
$ git clone git@github.com:Bloodevil/sony_camera_api.git
```

Then add the path of the module to the `PYTHONPATH` variable, with:

```bash
export PYTHONPATH="${PYTHONPATH}:/<download>/<path>/sony_camera_api/src"
```

Append this to the .bashrc file.

**Note**: pysony release 0.1.11, will not work with this program.


### Download and build sony_cam package

Use the following commands to get and build the ROS node:

```bash
$ git clone git@github.com:arcoslab/ros_sony_cam.git
$ cd ros_sony_cam/
$ catkin_make
```
Then append the following command to .bashrc file:

```bash
source /<full>/<path>/devel/setup.bash
```

## Run

Connect to camera's wireless access point with the password given on the camera's battery lid and then we may run the launch file, `pic_liveview_test`, to see the camera's output, liveview (low quality images) and high quality photos.

```bash
$ roslaunch sony_cam pic_liveview_test.launch
```


### Debug

When running with the roslaunch program it is hard to find errors in the setup.

You may perform the roslaunch tasks manually.

First, launch roscore:

```bash
$ roscore
```

Then in another terminal, launch main node:

```bash
$ rosrun sony_cam sony_cam_node.py
```

Open another terminal and run the following command to visualize low quality images with the image_view package:

```bash
$ rosrun image_view image_view image:=/liveview _image_transport:=compressed
```

The HD pictures are taken on request basis, so you need to launch the `client_test.py` node, which instructs the main node to take HD pictures.

```bash
$ rosrun sony_cam client_test.py
```

Then launch another image_view instance, to visualize the HD pictures:

```bash
$ rosrun image_view image_view image:=/hdpicture _image_transport:=compressed
```


## API Reference: sony_cam_node.py

### Liveview
The camera sends repeatedly, jpeg pictures of low quality, these are published in **liveview/compressed** topic, as **CompressedImage** type messages.


### Take photo
To obtain a high quality picture, make a request to service **sony_cam/request_image**, with a service of the type GetPolledImage, then the image will be published in **hdpicture/compressed** topic, as **CompressedImage** type messages. One service request must be made for every picture.


### Timeouts

Modify the following constants in sony_cam_node.py to set time limit for specific tasks:

 - TIMEOUT_TAKEPIC
 - TIMEOUT_FINDCAM
 - TIMEOUT_GETLIVEVIEW


## Examples

See `client_test.py` to learn how to use the service image request.


## License

GPLv3, see <http://www.gnu.org/licenses/>

## Contact

Alexander Marin Drobinoga <alexanderm2230@gmail.com>
