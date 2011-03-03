 # This message contains several messages useful for tasks such as object
# detection, grasping, etc
# 
# image, point cloud, regions of interest
#
###########################################

# The original 2D image
sensor_msgs/Image image_orig
# The processed 2D image
sensor_msgs/Image image_proc

# Point cloud
sensor_msgs/PointCloud2 cloud

# Region of interest (subwindow of full camera resolution), if applicable
sensor_msgs/RegionOfInterest[] roi

# Table hull
sensor_msgs/PointCloud2 table_hull

