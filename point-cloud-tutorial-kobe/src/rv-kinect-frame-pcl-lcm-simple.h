/*
 * kinect-frame-pcl.h
 *
 *  Created on: Nov 11, 2014
 *      Author: drc
 */

#ifndef RV_KINECT_FRAME_PCL_H_
#define RV_KINECT_FRAME_PCL_H_

#include <stdio.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <istream>

// may need to set the correct path
#include <pcl-1.7/pcl/io/io.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/features/integral_image_normal.h>
#include <pcl-1.7/pcl/common/transforms.h>
#include <pcl-1.7/pcl/features/integral_image_normal.h>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/features/normal_3d.h>

#include <pcl-1.7/pcl/search/organized.h>
#include <pcl-1.7/pcl/search/kdtree.h>
#include <pcl-1.7/pcl/features/normal_3d_omp.h>
#include <pcl-1.7/pcl/filters/conditional_removal.h>
#include <pcl-1.7/pcl/segmentation/extract_clusters.h>
#include <pcl-1.7/pcl/features/don.h>
#include <pcl-1.7/pcl/sample_consensus/sac_model_plane.h>
#include <pcl-1.7/pcl/sample_consensus/ransac.h>
#include <pcl-1.7/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.7/pcl/filters/extract_indices.h>


#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <GL/gl.h>

#include <lcmtypes/bot_core_image_t.h>
#include <lcmtypes/kinect_frame_msg_t.h>
#include <lcmtypes/obstacle_haptic_array_t.h>
#include <lcmtypes/obstacle_tts_t.h>
#include <lcmtypes/kinect_segmentlist_t.h>

#include <kinect/kinect-utils.h>

#include <cv_bridge_lcm/rv-cv-bridge-lcm.h>

#include "rv-kinect-frame-pcl-utils.h"
#include "rv-cv-pcl-utils.h"
#include "haptic-array-generator.h"

class blKinectFramePCL
{
public:
	blKinectFramePCL(lcm_t* lcm);
	~blKinectFramePCL();

	void on_frame(const kinect_frame_msg_t* msg);
	static void on_kinect_frame_aux(const lcm_recv_buf_t* rbuf,
			const char* channel,
			const kinect_frame_msg_t* msg,
			void* user_data);

	void set_pitch(float pitch);
	void set_height(float height);
	void set_fov(int h_fov, int v_fov);
	void set_decimate(int decimate);
	void set_haptic_settings(float h_min_r, float h_max_r,
			char* h_jn, int vibration_pattern);
	void set_haptic_array_mode(int mode);
	void set_motor_num(int num);
	void set_show_spotlight(int i);
	void set_publish_lcmgl(int i);
	void set_patch_density(float patch_density);
	void set_rotated(int is_rotated);
	void set_adjust_ground_height_time(int adjust_ground_height_time);
	void set_braille_display_mode(int mode);

	void publish_haptic_array(std::vector<int> haptic_array);

private:

	lcm_t* lcm_;

	int is_init;
	long timestamp_init;

	// default depth camera setting
	// we need to rotate the cloud to a rough MW
	float default_pitch;
	float default_height;	// no need... to be removed

	// estimate ground height and rotation
	// we assume the system sees ground in the first 5 seconds
	int64_t start_time;
	int64_t adjust_ground_height_time;
	// estimated ground height if
	float curr_ground_height;
	Eigen::Vector3f curr_ground_nor;
	Eigen::Matrix<double, 3, 4> curr_proj_mx;

	int h_fov;
	int v_fov;
	int publish_lcmgl;

	int haptic_array_mode;
	int motor_num;
	float patch_density;

	float haptic_min_range;
	float haptic_max_range;
	std::vector<int> haptic_just_notice_array;
	int haptic_vibration_pattern;

	int braille_display_mode;

	int ml_k;				// number of classes {wall, chair, table, ...}
	int ml_d;				// number of feature vector dim
	Eigen::MatrixXf ml_w;	// weights/parameters 	[k x d]
	Eigen::MatrixXf ml_b;	// bias 				[k x 1]
	std::vector<std::string> ml_lbls;	// {wall, chair, table, ...}

    KinectCalibration* kcal;
    int kinect_decimate;
    int is_rotated;

    cvBridgeLCM* cv_bridge;

    //BotTrans *transform;
    //TrajectoryLibrary* traj_lib;

	bot_lcmgl_t* lcmgl_;
	bot_lcmgl_t* lcmgl_spotlight;
	bot_lcmgl_t* lcmgl_haptic;
	bot_lcmgl_t* lcmgl_v;
	bot_lcmgl_t* lcmgl_h;
	bot_lcmgl_t* lcmgl_pointcloud;
	kinect_segmentlist_t* segment =new kinect_segmentlist_t();
};

#endif /* KINECT_FRAME_PCL_H_ */
