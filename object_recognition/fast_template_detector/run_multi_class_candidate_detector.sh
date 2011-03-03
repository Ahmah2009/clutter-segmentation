#rosrun fast_template_detector multi_class_candidate_detector $1 $2 /camera_info:=/narrow_stereo/left/camera_info /image:=/narrow_stereo/left/image_raw 
rosrun fast_template_detector multi_class_candidate_detector $1 $2 /camera_info:=/wide_stereo/right/camera_info /image:=/wide_stereo/right/image_rect_color

