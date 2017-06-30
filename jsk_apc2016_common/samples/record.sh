#!/bin/bash

set -x

rosbag record \
	/robot/joint_states /tf \
	/publish_bin_boxes/output /publish_tote_boxes/output \
	/right_hand_camera/left/rgb/image_raw/compressed /right_hand_camera/left/rgb/camera_info /right_hand_camera/left/depth_registered/image_raw/compressedDepth /right_hand_camera/left/depth_registered/camera_info \
	/right_hand_camera/right/rgb/image_raw/compressed /right_hand_camera/right/rgb/camera_info /right_hand_camera/right/depth_registered/image_raw/compressedDepth /right_hand_camera/right/depth_registered/camera_info \
	-O spam.bag

set +x
