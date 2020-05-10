#ifndef COMMON_H
#define COMMON_H

#endif // COMMON_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#endif

namespace autoCam_pkg {
/* Image streams for cameras */
extern cv_bridge::CvImagePtr my_global_cv_ptr;
extern cv_bridge::CvImagePtr my_global_cv_ptr_2;

/* Camera robot */
extern geometry_msgs::Pose hand_camera_pose;

}
