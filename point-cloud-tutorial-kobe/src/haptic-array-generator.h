/*
 * haptic-array-generator.h
 *
 *  Created on: Mar 10, 2015
 *      Author: drc
 */

#ifndef HAPTIC_ARRAY_GENERATOR_H_
#define HAPTIC_ARRAY_GENERATOR_H_

#include <stdio.h>
#include <iostream>
#include <string>
#include <cstddef>
#include <cmath>
#include <cfloat>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <map>
#include <string>

enum MapType {
	BL_HAPTIC_MAPPING_RANDOM,
	BL_HAPTIC_MAPPING_DEPTH_GRID,
	BL_HAPTIC_MAPPING_POLAR_2D,
	BL_HAPTIC_MAPPING_HORI_SURFACE_3D
};

namespace rv {

int get_range_value(float v_in);

std::vector<int> get_haptic_random(int n);

void get_haptic_range_blocks(
		cv::Mat im_dist, std::vector<cv::Rect> blocks,
		std::vector<float> & ranges,
		std::vector<float> & valid_pt_ratios);

std::vector<int> get_haptic_range_grids(
		cv::Mat im_h, cv::Mat im_dist, cv::Mat im_z, int decimate = 2);
std::vector<cv::Rect> get_haptic_range_grids_bboxes(std::vector<int> grids,
		int rows = 240, int cols = 320, int s = 10);

std::vector<int> get_haptic_polar_grids(cv::Mat im_g, int n,
		float patch_density);
std::vector<cv::Point> get_haptic_polar_grids_pts(cv::Mat im_g, int n,
		float patch_density);
void get_haptic_polar_grids_pts_3d(cv::Mat im_g, int n,
		std::vector<cv::Point> &grid_pts,
		std::vector<Eigen::Vector3d> &grid_pts_3d,
		float patch_density);

void get_occupancy_grid_polar_pts(cv::Mat im_occupancy,
		std::vector<cv::Point> & dense_ground_pts,
		std::vector<cv::Point> & hit_pts,
		std::vector<int> & is_hits,
		std::vector<int> & enter_dense_zones,
		int n, float patch_density);


std::vector<int> get_haptic_depth_grid(cv::Mat depth_map, int rows, int cols);


std::vector<cv::Rect> get_haptic_array_blocks(cv::Mat im, int rows, int cols);

std::vector<cv::Rect> get_haptic_array_blocks(cv::Mat im, int h_fov);

cv::Point2f get_end_pt_2d(Eigen::Vector3d end_pt_3d,
		float default_pitch, float default_height);

cv::Rect get_region_proposal(
		std::vector<Eigen::Vector3d> pts_cube,
		float default_pitch, float default_height);

}


#endif /* HAPTIC_ARRAY_GENERATOR_H_ */
