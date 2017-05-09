To run camera node: make sure you have usb_cam package
Run node:
	roslaunch bixi_vision usb_cam.launch

To run arrow_detection: make cfg files executable first
	==> cd ~/catkin_ws/src/Project_Bixi/bixi_vision/cfg
	==> chmod a+x building_blocks.cfg
	==> cd ~/catkin_ws
	==> catkin_make
Run node:
	roslaunch bixi_vision arrow.launch
