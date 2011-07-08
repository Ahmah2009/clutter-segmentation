/*
 * Copyright (c) 2010, Dejan Pangercic <pangercic@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ias_drawer_executive/OpenContainerAction.h>
#include <ias_drawer_executive/CloseContainerAction.h>
#include <ias_drawer_executive/OperateHandleController.h>
#include <ias_drawer_executive/RobotDriver.h>
#include <ias_drawer_executive/RobotArm.h>
#include <ias_drawer_executive/Torso.h>
#include <ias_drawer_executive/Gripper.h>
#include <ias_drawer_executive/Perception3d.h>

#include <pr2_gripper_sensor_msgs/PR2GripperGrabAction.h>
#include <visualization_msgs/Marker.h>
#include <ias_table_msgs/TableCluster.h>
#include <tf/transform_listener.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/common/common.h"
#include "pcl_ros/transforms.h"
#include "geometry_msgs/PoseStamped.h"
#include "clutseg/ClutsegObject.h"

void closeGripperComp(std::string side, double gain)
{
  typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperGrabAction>    GrabAC;
  GrabAC grab("/"+ side + "_gripper_sensor_controller/grab", true);
  while(!grab.waitForServer())
    {
      ROS_INFO("Waiting for the /%s_gripper_sensor_controller/grab to come up", side.c_str());
    }
  pr2_gripper_sensor_msgs::PR2GripperGrabGoal grab_goal;
  grab_goal.command.hardness_gain=gain;
  ROS_INFO("sending grab goal");
  grab.sendGoal(grab_goal);
  grab.waitForResult(ros::Duration(5.0));
  if (grab.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("/%s_gripper_sensor_controller/grab SUCCEEDED", side.c_str());
  return;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "grasp_object");
    ros::NodeHandle nh;
    tf::TransformListener tf;

    //hands in prep pose
    OperateHandleController::plateAttackPose();

    //0 - right, 1 - left
    int side = 1;
    Gripper::getInstance(side)->open();
    ROS_INFO("[grasp_object: ] Gripper opened.");

    sensor_msgs::PointCloud2 object_cloud, object_cloud_transformed;
    
    //get the object from clutsegmenter_node
    ros::ServiceClient client = nh.serviceClient<clutseg::ClutsegObject>("/clutsegmenter_node/clutseg_inliers");
    clutseg::ClutsegObject srv;
    if (client.call(srv))
      {
	ROS_INFO("[grasp_object: ] Calling </clutsegmenter_node/clutseg_inliers> service succeeded");
	object_cloud = srv.response.object;
      }
    else
      {
	ROS_ERROR("[grasp_object: ] Calling </clutsegmenter_node/clutseg_inliers> service failed");
	return 1;
      }

    //transform cloud to base link
    bool found_transform = tf.waitForTransform(object_cloud.header.frame_id, "base_link",
                                                object_cloud.header.stamp, ros::Duration(10.0));
    if (found_transform)
      {
	tf::StampedTransform transform;
	tf.lookupTransform("base_link", object_cloud.header.frame_id, object_cloud.header.stamp, transform);
	pcl_ros::transformPointCloud("base_link", transform, object_cloud, object_cloud_transformed);
      }
    else
      {
	ROS_ERROR("[grasp_object: ] No transform found!!!");
      }
	
    //Calculate the centroid of the object
    pcl::PointXYZ center, point_min, point_max;
    pcl::PointCloud<pcl::PointXYZ> object_cloud_pcl;
    pcl::fromROSMsg(object_cloud_transformed, object_cloud_pcl);
    pcl::getMinMax3D (object_cloud_pcl, point_min, point_max);
    center.x = (point_max.x + point_min.x)/2;
    center.y = (point_max.y + point_min.y)/2;
    center.z = (point_max.z + point_min.z)/2;

    ROS_INFO("Closing to %f", fabs(point_max.y - point_min.y));

    //grasp the object
    double obj_z_grasp_correction = 0.2 * fabs(point_max.z - point_min.z);
    RobotArm::getInstance(side)->universal_move_toolframe_ik(center.x-0.1, center.y-0.01, center.z, 0.0, 0.0, 0.0, 1.0, "base_link");
    RobotArm::getInstance(side)->universal_move_toolframe_ik(center.x, center.y-0.01, center.z-obj_z_grasp_correction, 0.0, 0.0, 0.0, 1.0, "base_link");

    //close compliant
    closeGripperComp("l", 0.05);
    
    //move the object to the side
    RobotArm::getInstance(side)->universal_move_toolframe_ik(center.x, center.y, center.z-obj_z_grasp_correction+0.2, 0.0, 0.0, 0.0, 1.0, "base_link");
    OperateHandleController::plateTuckPose();

    //drop the object
    Gripper::getInstance(side)->open();
    return 0;
}
