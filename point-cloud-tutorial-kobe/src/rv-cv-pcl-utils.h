/**
 * @file util.h
 * @brief Monocular SLAM with April tags and iSAM
 * @author: Michael Kaess
 *
 */
#include <stdio.h>
#include <iostream>
#include <string>
#include <cstddef>
#include <cmath>
#include <cfloat>

//#include <pcl-1.7/pcl/io/io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/features/integral_image_normal.h>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <map>
#include <string>

namespace rv {

//////////////////////////////////////////////////////////////////////
// find features for point cloud
//////////////////////////////////////////////////////////////////////

Eigen::Vector3i get_xyz_to_grid(Eigen::Vector3f p,
		int m_x = 6, int m_y = 6, int m_z = 4,
		int g_x = 300, int g_y = 300);

// im_n: normal
// im_v: vertical
// im_h: horizontal
// im_dist: distance
// im_z: height from the ground
//int get_surface_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
//		cv::Mat &im_n, cv::Mat &im_v, cv::Mat &im_h,
//		cv::Mat &im_dist, cv::Mat &im_z,
//		float scale = 0.04f);
int get_surface_normals(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::Normal>::Ptr normals,
		cv::Mat &im_n, cv::Mat &im_v, cv::Mat &im_h,
		cv::Mat &im_dist, cv::Mat &im_z);

int get_ground_surface(
		cv::Mat im_v, cv::Mat im_h, cv::Mat im_z, cv::Mat im_dist, cv::Mat im_n,
		cv::Mat &im_stixel, Eigen::Vector3f &est_nor);

void get_ground_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat im_ground,
		float & height_avg, float & height_std, std::vector<float> & block_heights);

int get_flat_surface(cv::Rect bbox, cv::Mat im_n, cv::Mat im_v);

// im_m: mask (all im above are the cloud size)
// im_g, g1: bird's view (300 x 300)
void get_occupancy_grid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Mat im_v, cv::Mat im_h, cv::Mat im_ground,
		cv::Mat &im_occupancy);

void get_occupancy_grid_view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Mat im_v, cv::Mat im_h,
		cv::Mat &im_m, cv::Mat &im_g, cv::Mat &im_g1);

void get_relative_transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Mat im_ground, int decimate,
		double fx, double fy, double px, double py,
		Eigen::Matrix<double, 3, 4> &P);

void get_relative_obj_img_corners(
		Eigen::Matrix<double, 3, 4> P,
		std::vector<Eigen::Vector3d> obj_corners,
		std::vector<cv::Point> &img_corners);

void get_cube_corners(Eigen::Vector3d tl, Eigen::Vector3d cube_dim,
		std::vector<Eigen::Vector3d> & corners);

void draw_cube_img_corners(cv::Mat &im_vis, std::vector<cv::Point> pts);

void get_nor_height_vec(cv::Rect box, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Mat im_v, cv::Mat im_h, cv::Mat im_ground,
		cv::Mat & nor_height_vec,
		float height_min = -0.35, float height_step = 0.2, int height_max_idx = 18);

//////////////////////////////////////////////////////////////////////
// RGB <--> Depth
//////////////////////////////////////////////////////////////////////

// search a shifted point with real-world dimension using point cloud
// used to get ground truth rectangle
void get_shifted_point(cv::Point src_pt, cv::Rect search_rect,
		float offset_x_m, float offset_y_m,
		cv::Mat im_rgb_rect, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Point & p_shifted_2d, Eigen::Vector3d & p_shifted_3d);

// check the dimensions of im_rgb_rect and cloud and return a rect of cloud dim
// currently for tango im_rgb_rect 1280 x 720, cloud 320 x 240
cv::Rect get_scaled_rect(cv::Rect r,
			cv::Mat im_rgb_rect, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

cv::Point get_scaled_image_point(cv::Point p,
		cv::Mat im_rgb_rect, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

cv::Point get_scaled_cloud_point(cv::Point p,
		cv::Mat im_rgb_rect, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

Eigen::Vector3d get_cloud_3d_point(cv::Point p,
		cv::Mat im_rgb_rect, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// reject:
// 1) not on vertical surface
// 2) too small or too large
int check_spatial_prior(cv::Mat im, cv::Rect r, std::vector<Eigen::Vector3d> &pts,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Mat im_vertical, cv::Mat im_dist,
		double phy_size_min = 0.005, double phy_size_max = 0.02);

// group characters into words
void group_regions(std::vector<cv::Rect> regions,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<cv::Rect> &candidates);

void crop_cloud(cv::Mat im, std::vector<cv::Rect> r,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_r);

//--------------------------------------------------------------
//////////////////////////////////////////////////////////////////////
// Visualization
//////////////////////////////////////////////////////////////////////

void get_color_table(std::map<int, cv::Scalar> &color_table);

void draw_annotation(cv::Mat &im, cv::Rect bbox, std::string anno,
		cv::Scalar color, int shift_x = 0);

//------------------------------------------------------
/** comments from Ed Olson's code:
 *
 * Given a 3x3 homography matrix and the focal lengths of the
 * camera, compute the pose of the tag. The focal lengths should
 * be given in pixels. For example, if the camera's focal length
 * is twice the width of the sensor, and the sensor is 600 pixels
 * across, the focal length in pixels is 2*600. Note that the
 * focal lengths in the fx and fy direction will be approximately
 * equal for most lenses, and is not a function of aspect ratio.
 *
 * Theory: The homography matrix is the product of the camera
 * projection matrix and the tag's pose matrix (the matrix that
 * projects points from the tag's local coordinate system to the
 * camera's coordinate frame).
 *
 * [ h00 h01 h02 h03] = [ 1/fx 0     0 0 ] [ R00 R01 R02 TX ]
 * [ h10 h11 h12 h13] = [ 0    1/fy  0 0 ] [ R10 R11 R12 TY ]
 * [ h20 h21 h22 h23] = [ 0    0     1 0 ] [ R20 R21 R22 TZ ]
 *                                         [ 0   0   0   1  ]
 *
 * When observing a tag, the points we project in world space all
 * have z=0, so we can form a 3x3 matrix by eliminating the 3rd
 * column of the pose matrix.
 *
 * [ h00 h01 h02 ] = [ 1/fx 0     0 0 ] [ R00 R01 TX ]
 * [ h10 h11 h12 ] = [ 0    1/fy  0 0 ] [ R10 R11 TY ]
 * [ h20 h21 h22 ] = [ 0    0     1 0 ] [ R20 R21 TZ ]
 *                                      [ 0   0   1  ]
 *
 * (note that these h's are different from the ones above.)
 *
 * We can multiply the right-hand side to yield a set of equations
 * relating the values of h to the values of the pose matrix.
 *
 * There are two wrinkles. The first is that the homography matrix
 * is known only up to scale. We recover the unknown scale by
 * constraining the magnitude of the first two columns of the pose
 * matrix to be 1. We use the geometric average scale. The sign of
 * the scale factor is recovered by constraining the observed tag
 * to be in front of the camera. Once scaled, we recover the first
 * two colmuns of the rotation matrix. The third column is the
 * cross product of these.
 *
 * The second wrinkle is that the computed rotation matrix might
 * not be exactly orthogonal, so we perform a polar decomposition
 * to find a good pure rotation approximation.
 *
 * Tagsize is the size of the tag in your desired units. I.e., if
 * your tag measures 0.25m along the side, your tag size is
 * 0.25. (The homography is computed in terms of *half* the tag
 * size, i.e., that a tag is 2 units wide as it spans from -1 to
 * +1, but this code makes the appropriate adjustment.)
 **/
Eigen::Matrix4d homographyToPose(double fx, double fy, double tag_size, Eigen::Matrix3d h);

}

