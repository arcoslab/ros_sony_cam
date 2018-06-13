## Contents

* src:
    * **client\_test.p**y: example client to test request\_image service.
    * **sony\_cam\_node.py**: ROS node to manage the camera, publishes liveview and offers the take picture service.

* launch:
    * **pic_liveview_test.launch**: launch file to test node functions, starts image_view nodes to preview published images.
    
* misc:
    * **available_api_qx1.txt**: available api functions in QX1 camera.


## Tests

To test the liveview and request_image service, use:

```bash
$ roslaunch sony\_cam pic\_liveview\_test.launch
```
This starts two image_view nodes to preview the high quality and the liveview images.
