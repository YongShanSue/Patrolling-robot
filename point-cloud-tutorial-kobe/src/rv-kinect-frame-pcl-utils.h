/*
 * pcl_lcmgl_vis.h
 *
 *  Created on: Jun 23, 2014
 *      Author: drc
 */

#ifndef KINECT_FRAME_PCL_UTILS_H_
#define KINECT_FRAME_PCL_UTILS_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <GL/gl.h>

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/pcl_macros.h>
#include <pcl-1.7/pcl/pcl_base.h>
#include <pcl-1.7/pcl/PointIndices.h>

#include <opencv2/opencv.hpp>

#include <lcmtypes/bot_core_image_t.h>
#include <lcmtypes/kinect_frame_msg_t.h>
#include <kinect/kinect-utils.h>

#include <jpeg-utils/jpeg-utils.h>
#include <jpeg-utils/jpeg-utils-ijg.h>

#include <zlib.h>

#define DEPTH_VAL 8192

namespace rv {


///////////////////////////////////////////////////////////////
// visualization

void draw_cloud_ptr_lcmgl(bot_lcmgl_t* lcmgl_, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		float r = 1.0, float g = 0.0, float b = 1.0);

void draw_cloud_ptr_lcmgl(bot_lcmgl_t* lcmgl_, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Mat im_mask, int haptic_mode = 3);

void draw_eigen_planes_lcmgl(bot_lcmgl_t* lcmgl_, std::vector<Eigen::Vector3f> planes);

void draw_corners_lcmgl(bot_lcmgl_t* lcmgl_, std::vector<Eigen::Vector3f> corners,
		double r, double g, double b);

void draw_pose_lcmgl(bot_lcmgl_t* lcmgl_, Eigen::Matrix4d pose,
		float r=1, float g=0, float b=1);

Eigen::Vector3d get_3d_corner(Eigen::Matrix4d pose, double x, double y, double z);

void draw_pose_spotlight_lcmgl(bot_lcmgl_t* lcmgl_, Eigen::Matrix4d pose, float fov_h = 60, float fov_v = 40);

void draw_tile_lcmgl(bot_lcmgl_t* lcmgl_, Eigen::Matrix4d pose, float tile_size = 0.6);

void draw_brid_view_range_lcmgl(bot_lcmgl_t* lcmgl_, Eigen::Matrix4d pose, float fov_h = 60, float fov_v = 40);

void draw_line_lcmgl(bot_lcmgl_t* lcmgl_, Eigen::Vector3d st_pt, Eigen::Vector3d end_pt,
		float r = 0, float g = 0, float b = 1);

void draw_cube_lcmgl(bot_lcmgl_t* lcmgl_, std::vector<Eigen::Vector3d> obj_corners,
		float r = 0, float g = 0, float b = 1);

void draw_texture_lcmgl(bot_lcmgl_t* lcmgl_, cv::Mat im_text, std::vector<Eigen::Vector3d> label_corners_3d);

cv::Mat get_annotation_img(std::string anno, cv::Scalar color, double fontScale, int thickness, int adjust_h);

void draw_lcmgl_label(std::string label_text, std::vector <Eigen::Vector3d> bbox_corners,
		Eigen::Vector2d bbox_size, bot_lcmgl_t* lcmgl_, cv::Scalar color);

void draw_cloud_lcmgl(bot_lcmgl_t* lcmgl_, pcl::PointCloud<pcl::PointXYZRGB> chull,
		float r, float g, float b);

void draw_motor_signals_lcmgl(bot_lcmgl_t* lcmgl, std::vector<int> haptic_array);

void draw_maze(bot_lcmgl_t* lcmgl, std::vector<std::vector<float> > maze_segments);

////////////////////////////////////////////////////////////////
// unpack
void get_default_kinect_calib(KinectCalibration* kcal);

void get_default_ti_board_calib(KinectCalibration* kcal);

void unpack_kinect_frame(const kinect_frame_msg_t *msg, uint8_t* rgb_data,
		KinectCalibration* kcal, int kinect_decimate,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

void unpack_kinect_frame_depth_vis(const kinect_frame_msg_t *msg,
		uint8_t* depth_img);
}



#endif /* KINECT_FRAME_PCL_UTILS_H_ */
