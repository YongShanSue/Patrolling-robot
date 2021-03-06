/*
 * pcl_lcmgl_vis.cpp
 *
 *  Created on: Jun 23, 2014
 *      Author: drc
 */
#include "rv-kinect-frame-pcl-utils.h"

using namespace std;
using namespace cv;

namespace rv {

void draw_cloud_ptr_lcmgl(bot_lcmgl_t* lcmgl_, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		float r, float g, float b) {

	bot_lcmgl_enable(lcmgl_, GL_BLEND);
	bot_lcmgl_point_size(lcmgl_, 2.0f);

	bot_lcmgl_begin(lcmgl_, GL_POINTS);
	//cout << cloud->points.size() << endl;
	for(int i = 0; i < cloud->points.size(); i++){

		if (cloud->points[i].z < 0.5){
			bot_lcmgl_color4f(lcmgl_, 0, 1, 0, 0.2);
		}else{
			bot_lcmgl_color4f(lcmgl_, 1, 0, 0, 0.2);
		}

		bot_lcmgl_vertex3f(lcmgl_,
				cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

	}
	bot_lcmgl_end(lcmgl_);

}

void draw_cloud_ptr_lcmgl(bot_lcmgl_t* lcmgl_,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Mat im_mask, int haptic_mode) {


	// check cloud has the same dimentions with im_mask
	if(cloud->height != im_mask.rows || cloud->width != im_mask.cols){
		cerr << "DIMENTIONS DO NOT MATCH" << endl;
		cout << "height: " << cloud->height << " " << im_mask.rows << endl;
		cout << "width: " << cloud->width << " " << im_mask.cols << endl;
	}

	bot_lcmgl_enable(lcmgl_, GL_BLEND);
	bot_lcmgl_point_size(lcmgl_, 2.0f);
	bot_lcmgl_begin(lcmgl_, GL_POINTS);
	//cout << cloud->points.size() << endl;
	for(int i = 0; i < cloud->height; i++){
		for(int j = 0; j < cloud->width; j++){

			if (im_mask.at<uchar>(i, j) == 0){
				continue;
			}

			// ground is green
			if (im_mask.at<uchar>(i, j) == 255){
				bot_lcmgl_color4f(lcmgl_, 0, 1, 0, 0.2);
			}else{

				if(haptic_mode == 2){
					// everything not ground is obstacle
					if(im_mask.at<uchar>(i, j) >= 1){
						bot_lcmgl_color4f(lcmgl_, 1, 0, 0, 0.2);
					}
				}else if(haptic_mode == 3){
					if(im_mask.at<uchar>(i, j) == 2){
						bot_lcmgl_color4f(lcmgl_, 0, 1, 1, 0.2);
					}else if(im_mask.at<uchar>(i, j) == 3){
						bot_lcmgl_color4f(lcmgl_, 1, 1, 0, 0.2);
					}else if(im_mask.at<uchar>(i, j) == 1){
						bot_lcmgl_color4f(lcmgl_, 1, 0, 0, 0.2);
					}
				}else {
					bot_lcmgl_color4f(lcmgl_, 0, 0, 1, 0.2);
				}
			}



			bot_lcmgl_vertex3f(lcmgl_,
					cloud->at(j, i).x, cloud->at(j, i).y, cloud->at(j, i).z);
		}
	}
	bot_lcmgl_end(lcmgl_);

}

void draw_eigen_planes_lcmgl(bot_lcmgl_t* lcmgl_, std::vector<Eigen::Vector3f> planes) {

	for (int ii = 0; ii < planes.size(); ++ii) {
		bot_lcmgl_color3f(lcmgl_, 0, 0, 1);
		bot_lcmgl_vertex3f(lcmgl_,
				planes[ii][0], planes[ii][1], planes[ii][2]);
	}
}

void draw_corners_lcmgl(bot_lcmgl_t* lcmgl_, vector<Eigen::Vector3f> corners,
		double r, double g, double b){

	bot_lcmgl_translated(lcmgl_, 0, 0, 0);
	bot_lcmgl_line_width(lcmgl_, 2.0f);
	bot_lcmgl_point_size(lcmgl_, 10.0f);
	bot_lcmgl_begin(lcmgl_, GL_POINTS);
	for(int i = 0; i < corners.size(); i++){

		bot_lcmgl_color3f(lcmgl_, r, g, b);
		bot_lcmgl_vertex3f(lcmgl_, corners[i][0], corners[i][1], corners[i][2]);
	}
	bot_lcmgl_end(lcmgl_);


}

void draw_pose_lcmgl(bot_lcmgl_t* lcmgl_, Eigen::Matrix4d pose,
		float r, float g, float b){

	float normal_length = 0.6;

	bot_lcmgl_translated(lcmgl_, 0, 0, 0);
	bot_lcmgl_line_width(lcmgl_, 4.0f);
	bot_lcmgl_point_size(lcmgl_, 7.0f);
	bot_lcmgl_color3f(lcmgl_, r, g, b);

	Eigen::Vector3d pt;
	pt << pose(0, 3), pose(1, 3), pose(2, 3);

	Eigen::Matrix4d M_fd, M_left, M_up;
	Eigen::Matrix4d T_fd, T_left, T_up;
	T_fd << 1, 0, 0, normal_length,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	M_fd = pose * T_fd;

	T_left << 	1, 0, 0, 0,
				0, 1, 0, normal_length/2,
				0, 0, 1, 0,
				0, 0, 0, 1;
	M_left = pose * T_left;

	T_up << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, normal_length/2,
			0, 0, 0, 1;
	M_up = pose * T_up;

	Eigen::Vector3d pt_fd, pt_left, pt_up;
	pt_fd << M_fd(0, 3), M_fd(1, 3), M_fd(2, 3);
	pt_left << M_left(0, 3), M_left(1, 3), M_left(2, 3);
	pt_up << M_up(0, 3), M_up(1, 3), M_up(2, 3);

	bot_lcmgl_begin(lcmgl_, GL_LINE_STRIP);
	bot_lcmgl_vertex3f(lcmgl_, pt[0], pt[1], pt[2]);
	bot_lcmgl_vertex3f(lcmgl_, pt_fd[0], pt_fd[1], pt_fd[2]);
	bot_lcmgl_end(lcmgl_);

	bot_lcmgl_begin(lcmgl_, GL_LINE_STRIP);
	bot_lcmgl_vertex3f(lcmgl_, pt[0], pt[1], pt[2]);
	bot_lcmgl_vertex3f(lcmgl_, pt_left[0], pt_left[1], pt_left[2]);
	bot_lcmgl_end(lcmgl_);

	bot_lcmgl_begin(lcmgl_, GL_LINE_STRIP);
	bot_lcmgl_vertex3f(lcmgl_, pt[0], pt[1], pt[2]);
	bot_lcmgl_vertex3f(lcmgl_, pt_up[0], pt_up[1], pt_up[2]);
	bot_lcmgl_end(lcmgl_);

	bot_lcmgl_begin(lcmgl_, GL_POINTS);
	bot_lcmgl_vertex3f(lcmgl_, pt[0], pt[1], pt[2]);
	bot_lcmgl_end(lcmgl_);
}


Eigen::Vector3d get_3d_corner(Eigen::Matrix4d pose, double x, double y, double z){
	Eigen::Matrix4d M_tl, T_tl;
	T_tl << 1, 0, 0, x,
			0, 1, 0, y,
			0, 0, 1, z,
			0, 0, 0, 1;
	M_tl = pose * T_tl;
	Eigen::Vector3d c_tl;
	c_tl << M_tl(0, 3), M_tl(1, 3), M_tl(2, 3);
	return c_tl;
}

void draw_pose_spotlight_lcmgl(bot_lcmgl_t* lcmgl_, Eigen::Matrix4d pose, float fov_h, float fov_v){

	double fw = 4;

	bot_lcmgl_line_width(lcmgl_, 3.0f);
	bot_lcmgl_point_size(lcmgl_, 10.0f);

	bot_lcmgl_enable(lcmgl_, GL_BLEND);

	// position
	Eigen::Vector3d pos_3d;
	pos_3d << pose(0, 3), pose(1, 3), pose(2, 3);

	vector<Eigen::Vector3d> corners;
	Eigen::Matrix4d M_tl, M_tr, M_br, M_bl;
	Eigen::Matrix4d T_tl, T_tr, T_br, T_bl;

	double y = fw * tan(fov_h/2 / 180.0 * M_PI);
	double z = fw * tan(fov_v/2 / 180.0 * M_PI);
	Eigen::Vector3d c_tl = get_3d_corner(pose, fw, y, z);
	Eigen::Vector3d c_tr = get_3d_corner(pose, fw, -y, z);
	Eigen::Vector3d c_br = get_3d_corner(pose, fw, -y, -z);
	Eigen::Vector3d c_bl = get_3d_corner(pose, fw, y, -z);
	corners.push_back(c_tl);
	corners.push_back(c_tr);
	corners.push_back(c_br);
	corners.push_back(c_bl);

	for(int i = 0; i < 4; i++){
		int j;
		if (i < 3){
			j = i + 1;
		}else{
			j = 0;
		}
		bot_lcmgl_color4f(lcmgl_, 1, 1, 0, 0.2);
		bot_lcmgl_begin(lcmgl_, GL_TRIANGLES);
		bot_lcmgl_vertex3d(lcmgl_, pos_3d[0], pos_3d[1], pos_3d[2]);
		bot_lcmgl_vertex3d(lcmgl_,corners[i][0], corners[i][1], corners[i][2]);
		bot_lcmgl_vertex3d(lcmgl_,corners[j][0], corners[j][1], corners[j][2]);
		bot_lcmgl_end(lcmgl_);

		bot_lcmgl_color4f(lcmgl_, 1, 0, 0, 0.2);
		bot_lcmgl_begin(lcmgl_, GL_LINE_STRIP);
		bot_lcmgl_vertex3d(lcmgl_, pos_3d[0], pos_3d[1], pos_3d[2]);
		bot_lcmgl_vertex3d(lcmgl_,corners[i][0], corners[i][1], corners[i][2]);
		bot_lcmgl_vertex3d(lcmgl_,corners[j][0], corners[j][1], corners[j][2]);
		bot_lcmgl_vertex3d(lcmgl_, pos_3d[0], pos_3d[1], pos_3d[2]);
		bot_lcmgl_end(lcmgl_);
	}
}

void draw_tile_lcmgl(bot_lcmgl_t* lcmgl_, Eigen::Matrix4d pose, float tile_size){

	double m_x = tile_size;
	double m_y = tile_size;

	// position
	Eigen::Vector3d pos_3d;
	pos_3d << pose(0, 3), pose(1, 3), pose(2, 3);

	Eigen::Vector3d c_tl = get_3d_corner(pose, m_x, m_y / 2, 0);
	Eigen::Vector3d c_tr = get_3d_corner(pose, m_x, -m_y / 2, 0);
	Eigen::Vector3d c_br = get_3d_corner(pose, 0, -m_y / 2, 0);
	Eigen::Vector3d c_bl = get_3d_corner(pose, 0, m_y / 2, 0);

	bot_lcmgl_color4f(lcmgl_, 0, 0, 1, 0.2);
	bot_lcmgl_begin(lcmgl_, GL_LINE_STRIP);
	// ignore z
	bot_lcmgl_vertex3d(lcmgl_, c_tl[0], c_tl[1], 0);
	bot_lcmgl_vertex3d(lcmgl_, c_tr[0], c_tr[1], 0);
	bot_lcmgl_vertex3d(lcmgl_, c_br[0], c_br[1], 0);
	bot_lcmgl_vertex3d(lcmgl_, c_bl[0], c_bl[1], 0);
	bot_lcmgl_vertex3d(lcmgl_, c_tl[0], c_tl[1], 0);
	bot_lcmgl_end(lcmgl_);
}

void draw_brid_view_range_lcmgl(bot_lcmgl_t* lcmgl_, Eigen::Matrix4d pose, float fov_h, float fov_v){

	double m_x = 3;
	double m_y = 3;

	// position
	Eigen::Vector3d pos_3d;
	pos_3d << pose(0, 3), pose(1, 3), pose(2, 3);

	Eigen::Vector3d c_tl = get_3d_corner(pose, m_x, m_y / 2, 0);
	Eigen::Vector3d c_tr = get_3d_corner(pose, m_x, -m_y / 2, 0);
	Eigen::Vector3d c_br = get_3d_corner(pose, 0, -m_y / 2, 0);
	Eigen::Vector3d c_bl = get_3d_corner(pose, 0, m_y / 2, 0);

	// TODO: get pitch
	// find blind area
	double b_x = 1.7;
	Eigen::Vector3d b_tl = get_3d_corner(pose, b_x, m_y / 2, 0);
	Eigen::Vector3d b_tr = get_3d_corner(pose, b_x, -m_y / 2, 0);
	Eigen::Vector3d b_br = get_3d_corner(pose, 0, -m_y / 2, 0);
	Eigen::Vector3d b_bl = get_3d_corner(pose, 0, m_y / 2, 0);

	bot_lcmgl_color4f(lcmgl_, 0, 1, 1, 0.2);
	bot_lcmgl_begin(lcmgl_, GL_LINE_STRIP);
	// ignore z
	bot_lcmgl_vertex3d(lcmgl_, b_tl[0], b_tl[1], 0);
	bot_lcmgl_vertex3d(lcmgl_, b_tr[0], b_tr[1], 0);
	bot_lcmgl_vertex3d(lcmgl_, b_br[0], b_br[1], 0);
	bot_lcmgl_vertex3d(lcmgl_, b_bl[0], b_bl[1], 0);
	bot_lcmgl_vertex3d(lcmgl_, b_tl[0], b_tl[1], 0);
	bot_lcmgl_end(lcmgl_);

	bot_lcmgl_color4f(lcmgl_, 0, 0, 1, 0.2);
	bot_lcmgl_begin(lcmgl_, GL_LINE_STRIP);
	// ignore z
	bot_lcmgl_vertex3d(lcmgl_, c_tl[0], c_tl[1], 0);
	bot_lcmgl_vertex3d(lcmgl_, c_tr[0], c_tr[1], 0);
	bot_lcmgl_vertex3d(lcmgl_, c_br[0], c_br[1], 0);
	bot_lcmgl_vertex3d(lcmgl_, c_bl[0], c_bl[1], 0);
	bot_lcmgl_vertex3d(lcmgl_, c_tl[0], c_tl[1], 0);
	bot_lcmgl_end(lcmgl_);

}

void draw_line_lcmgl(bot_lcmgl_t* lcmgl_, Eigen::Vector3d st_pt, Eigen::Vector3d end_pt,
		float r, float g, float b){

	// position

	bot_lcmgl_line_width(lcmgl_, 4.0f);
	bot_lcmgl_point_size(lcmgl_, 7.0f);
	bot_lcmgl_color3f(lcmgl_, r, g, b);

	bot_lcmgl_begin(lcmgl_, GL_LINE_STRIP);
	bot_lcmgl_vertex3d(lcmgl_, st_pt[0], st_pt[1], st_pt[2]);
	bot_lcmgl_vertex3d(lcmgl_,end_pt[0], end_pt[1], end_pt[2]);
	bot_lcmgl_end(lcmgl_);

}

void draw_cube_lcmgl(bot_lcmgl_t* lcmgl_, std::vector<Eigen::Vector3d> obj_corners,
		float r, float g, float b){

	draw_line_lcmgl(lcmgl_, obj_corners[0], obj_corners[1], r, g, b);
	draw_line_lcmgl(lcmgl_, obj_corners[1], obj_corners[2], r, g, b);
	draw_line_lcmgl(lcmgl_, obj_corners[2], obj_corners[3], r, g, b);
	draw_line_lcmgl(lcmgl_, obj_corners[3], obj_corners[0], r, g, b);

	draw_line_lcmgl(lcmgl_, obj_corners[0], obj_corners[4], r, g, b);
	draw_line_lcmgl(lcmgl_, obj_corners[1], obj_corners[5], r, g, b);
	draw_line_lcmgl(lcmgl_, obj_corners[2], obj_corners[6], r, g, b);
	draw_line_lcmgl(lcmgl_, obj_corners[3], obj_corners[7], r, g, b);

	draw_line_lcmgl(lcmgl_, obj_corners[4], obj_corners[5], r, g, b);
	draw_line_lcmgl(lcmgl_, obj_corners[5], obj_corners[6], r, g, b);
	draw_line_lcmgl(lcmgl_, obj_corners[6], obj_corners[7], r, g, b);
	draw_line_lcmgl(lcmgl_, obj_corners[7], obj_corners[4], r, g, b);

}

void draw_texture_lcmgl(bot_lcmgl_t* lcmgl_, cv::Mat im_text, vector<Eigen::Vector3d> label_corners_3d){

	int texid1 = bot_lcmgl_texture2d(lcmgl_,
			im_text.data,
			im_text.cols,
			im_text.rows,
			im_text.cols * 3,
			BOT_LCMGL_RGB,
			BOT_LCMGL_UNSIGNED_BYTE,
			BOT_LCMGL_COMPRESS_NONE);

	bot_lcmgl_texture_draw_quad(lcmgl_, texid1,
			label_corners_3d[0][0], label_corners_3d[0][1], label_corners_3d[0][2],
			label_corners_3d[3][0], label_corners_3d[3][1], label_corners_3d[3][2],
			label_corners_3d[2][0], label_corners_3d[2][1], label_corners_3d[2][2],
			label_corners_3d[1][0], label_corners_3d[1][1], label_corners_3d[1][2]);

}

cv::Mat get_annotation_img(string anno, Scalar color, double fontScale, int thickness, int adjust_h){

	int fontFace = cv::FONT_HERSHEY_PLAIN;

	string text = anno;
//		string text = "180 pt";
	int baseline=0;
	Size text_size = getTextSize(text, fontFace, fontScale, thickness, &baseline);
	text_size += Size(0, 12);
	//int text_pos_y = min(3, (int) (0.05 * text_size.height));
	Point text_org(0, text_size.height - adjust_h);

	//cout << text_size.width << " " << text_size.width;

	Mat im_text = Mat::zeros(text_size, CV_8UC3);
	// draw the box
	rectangle(im_text, Point(0, 0),
			Point(text_size.width, text_size.height * 1.2),
			color, CV_FILLED);
	// then put the text itself
	putText(im_text, text, text_org, fontFace, fontScale,
	        Scalar::all(0), thickness, 8);

	return im_text;
}

void draw_lcmgl_label(string label_text, vector <Eigen::Vector3d> bbox_corners,
		Eigen::Vector2d bbox_size, bot_lcmgl_t* lcmgl_, Scalar color){
	// setting for text
	int fontFace = cv::FONT_HERSHEY_PLAIN;
	double fontScale = 24;
	int thickness = 20;

	string text = string(label_text);
	Mat im_text = get_annotation_img(text, color, fontScale, thickness, -15);
	cv::cvtColor(im_text, im_text, CV_RGB2BGR);

	//show_image(im_text);

	//cout << im_text.rows << " " << im_text.cols << endl;
	//		// get label 3d corners
	vector<Eigen::Vector3d> label_corners_3d;

	//vector <Eigen::Vector3d> text_spots_3d = get_text_spot_t_corners(&spot);
	Eigen::Vector3d delta_v1 = bbox_corners[1] - bbox_corners[0];
	Eigen::Vector3d delta_v2 = bbox_corners[3] - bbox_corners[0];

	// starting from lbl bottom left (aka text spot top left)
	Eigen::Vector3d pt_3d = bbox_corners[0];
	// top_left
	pt_3d -= delta_v2 * ((double)im_text.rows / (double)bbox_size[1]);
	label_corners_3d.push_back(pt_3d);
	// top_right
	pt_3d += delta_v1 * ((double)im_text.cols / (double)bbox_size[0]);
	label_corners_3d.push_back(pt_3d);
	// bottom right	show_image(im_text);

	pt_3d += delta_v2 * ((double)im_text.rows / (double)bbox_size[1]);
	label_corners_3d.push_back(pt_3d);
	// bottom left
	pt_3d -= delta_v1 * ((double)im_text.cols / (double)bbox_size[0]);
	label_corners_3d.push_back(pt_3d);

	// shift to the right
	for(int i = 0; i < label_corners_3d.size(); i++){
		label_corners_3d[i] += delta_v1 * 0.35;
		label_corners_3d[i] -= delta_v2 * 0.1;
	}

	//print_corners(label_corners_3d);

	draw_texture_lcmgl(lcmgl_, im_text.clone(), label_corners_3d);

}

void draw_cloud_lcmgl(bot_lcmgl_t* lcmgl_, pcl::PointCloud<pcl::PointXYZRGB> chull,
		float r, float g, float b) {

	bot_lcmgl_begin(lcmgl_, GL_POLYGON);

	bot_lcmgl_color4f(lcmgl_, r, g, b, 0.4);

	for (int ii = 0; ii < chull.points.size(); ++ii) {
		bot_lcmgl_vertex3f(lcmgl_, chull.points[ii].x,
				chull.points[ii].y,
				chull.points[ii].z);
	}
	bot_lcmgl_vertex3f(lcmgl_, chull.points[0].x,
			chull.points[0].y,
			chull.points[0].z);
	bot_lcmgl_end(lcmgl_);

}

void draw_motor_signals_lcmgl(bot_lcmgl_t* lcmgl, std::vector<int> haptic_array){

}

void draw_maze(bot_lcmgl_t* lcmgl, std::vector<std::vector<float> > maze_segments){

	for(int i = 0; i < maze_segments.size(); i++){

		Eigen::Vector3d st_pt, end_pt;
		//std::cout << maze_segments[i].size() << std::endl;
		st_pt << maze_segments[i][0], maze_segments[i][1], 0;
		end_pt << maze_segments[i][2], maze_segments[i][3], 0;

//		std::cout << "start:\t" << st_pt << std::endl;
//		std::cout << "end:\t" << end_pt << std::endl;
		draw_line_lcmgl(lcmgl, st_pt, end_pt,
				0.5, 0.5, 0);

	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

void get_default_kinect_calib(KinectCalibration* kcal){

	kcal->intrinsics_depth.fx = 576.09757860;
	kcal->intrinsics_depth.cx = 321.06398107;
	kcal->intrinsics_depth.cy = 242.97676897;
	kcal->intrinsics_rgb.fx = 576.09757860;
	kcal->intrinsics_rgb.cx = 321.06398107;
	kcal->intrinsics_rgb.cy = 242.97676897;
	kcal->intrinsics_rgb.k1 = 0; // none given so far
	kcal->intrinsics_rgb.k2 = 0; // none given so far
	kcal->shift_offset = 1079.4753;
	kcal->projector_depth_baseline = 0.07214;
	//double rotation[9];
	double rotation[]={0.999999, -0.000796, 0.001256, 0.000739, 0.998970, 0.045368, -0.001291, -0.045367, 0.998970};
	double depth_to_rgb_translation[] ={ -0.015756, -0.000923, 0.002316};
	memcpy(kcal->depth_to_rgb_rot, rotation, 9*sizeof(double));
	memcpy(kcal->depth_to_rgb_translation, depth_to_rgb_translation  , 3*sizeof(double));
}

void get_default_ti_board_calib(KinectCalibration* kcal){

	kcal->intrinsics_depth.fx = 876.09757860;
	kcal->intrinsics_depth.cx = 321.06398107;
	kcal->intrinsics_depth.cy = 242.97676897;

	kcal->intrinsics_rgb.fx = 576.09757860;
	kcal->intrinsics_rgb.cx = 321.06398107;
	kcal->intrinsics_rgb.cy = 242.97676897;
	kcal->intrinsics_rgb.k1 = 0; // none given so far
	kcal->intrinsics_rgb.k2 = 0; // none given so far
	kcal->shift_offset = 1079.4753;
	kcal->projector_depth_baseline = 0.07214;
	//double rotation[9];
	double rotation[]={0.999999, -0.000796, 0.001256, 0.000739, 0.998970, 0.045368, -0.001291, -0.045367, 0.998970};
	double depth_to_rgb_translation[] ={ -0.015756, -0.000923, 0.002316};
	memcpy(kcal->depth_to_rgb_rot, rotation, 9*sizeof(double));
	memcpy(kcal->depth_to_rgb_translation, depth_to_rgb_translation  , 3*sizeof(double));
}

static inline void
_matrix_vector_multiply_3x4_4d (const double m[12], const double v[4],
		double result[3])
{
	result[0] = m[0]*v[0] + m[1]*v[1] + m[2] *v[2] + m[3] *v[3];
	result[1] = m[4]*v[0] + m[5]*v[1] + m[6] *v[2] + m[7] *v[3];
	result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
}

void unpack_kinect_frame(const kinect_frame_msg_t *msg, uint8_t* rgb_data,
		KinectCalibration* kcal, int kinect_decimate,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){

	/////////////////////////////////////////////////////////////////////
	/// 1.1 RGB:
	// TODO check width, height
	if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB) {
		memcpy(rgb_data, msg->image.image_data,
				msg->depth.width * msg->depth.height * 3);
	} else if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG) {
		jpeg_decompress_8u_rgb (msg->image.image_data, msg->image.image_data_nbytes,
				rgb_data, msg->image.width, msg->image.height, msg->image.width* 3);
		jpegijg_decompress_8u_rgb(msg->image.image_data, msg->image.image_data_nbytes,
		        rgb_data, msg->image.width, msg->image.height, msg->image.width* 3);
	}

	/////////////////////////////////////////////////////////////////////
	/// 1.2. DEPTH:
	uint8_t* uncompress_buffer;
	int uncompress_buffer_size;
	const uint8_t* depth_data =NULL; //= msg->depth.depth_data;
	// 1.2.1 De-compress if necessary:
	if(msg->depth.compression != KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
		if(msg->depth.uncompressed_size > uncompress_buffer_size) {
			uncompress_buffer_size = msg->depth.uncompressed_size;
			uncompress_buffer = (uint8_t*) realloc(uncompress_buffer, uncompress_buffer_size);
		}
		unsigned long dlen = msg->depth.uncompressed_size;
		int status = uncompress(uncompress_buffer, &dlen,
				msg->depth.depth_data, msg->depth.depth_data_nbytes);
		if(status != Z_OK) {
			return;
		}
		depth_data =(uint8_t*) uncompress_buffer;
		
	}else{
		depth_data = (uint8_t*) msg->depth.depth_data;
	}

	int npixels = msg->depth.width * msg->depth.height;
	if (msg->depth.depth_data_format == KINECT_DEPTH_MSG_T_DEPTH_11BIT){
		/////////////////////////////////////////////////////////////////////
		// Freenect Data
		printf("Successfully log data another format\t");
		uint16_t* disparity_array = (uint16_t*) malloc(msg->depth.width * msg->depth.height * sizeof(uint16_t));
		if(G_BYTE_ORDER == G_LITTLE_ENDIAN) {
			int16_t* rdd = (int16_t*) depth_data;
			int i;
			for(i=0; i<npixels; i++) {
				int d = rdd[i];
				disparity_array[i] = d;
			}
		} else {
			fprintf(stderr, "Big endian systems not supported\n");
		}

		/// 2 Calculate transformation matrices:
		double depth_to_rgb_uvd[12];
		double depth_to_depth_xyz[16];
		kinect_calib_get_depth_uvd_to_rgb_uvw_3x4(kcal, depth_to_rgb_uvd);
		kinect_calib_get_depth_uvd_to_depth_xyz_4x4(kcal, depth_to_depth_xyz);
		double depth_to_depth_xyz_trans[16];
		//_matrix_transpose_4x4d(depth_to_depth_xyz, depth_to_depth_xyz_trans);
		bot_matrix_transpose_4x4d(depth_to_depth_xyz, depth_to_depth_xyz_trans);

		// 3 for each depth point find the corresponding xyz and then RGB
		//   then put into PCL structure
		cloud->width    =(int) (msg->depth.width/ (double) kinect_decimate) ;
		cloud->height   =(int) (msg->depth.height/ (double) kinect_decimate);
		
		cloud->is_dense = false;
		cloud->points.resize (cloud->width * cloud->height);
		double xyzw2[4];
		int j2=0;
		// NB: the order of these loop was changed... aug 2011. important
		for(int v=0; v<msg->depth.height; v=v+ kinect_decimate) { // t2b state->height 480
			for(int u=0; u<msg->depth.width; u=u+kinect_decimate ) {  //l2r state->width 640
				// 3.4.1 compute distorted pixel coordinates
				
				uint16_t disparity = disparity_array[v*msg->depth.width+u];
				double uvd_depth[4] = { (double)u, (double)v, (double)disparity, 1 };
				double uvd_rgb[3];
				_matrix_vector_multiply_3x4_4d(depth_to_rgb_uvd, uvd_depth, uvd_rgb);
				double uv_rect[2] = {
						uvd_rgb[0] / uvd_rgb[2],
						uvd_rgb[1] / uvd_rgb[2]
				};
				double uv_dist[2];
				kinect_calib_distort_rgb_uv(kcal, uv_rect, uv_dist);
				int u_rgb = uv_dist[0] + 0.5;
				int v_rgb = uv_dist[1] + 0.5;
				uint8_t r, g, b;
				if(u_rgb >= msg->depth.width || u_rgb < 0 || v_rgb >= msg->depth.height || v_rgb < 0) {
					r = g = b = 0;
				} else {
					r = rgb_data[v_rgb*msg->depth.width*3 + u_rgb*3 + 0];
					g = rgb_data[v_rgb*msg->depth.width*3 + u_rgb*3 + 1];
					b = rgb_data[v_rgb*msg->depth.width*3 + u_rgb*3 + 2];
				}
				// 3.4.2 find the xyz location of the points:
				bot_matrix_multiply(depth_to_depth_xyz, 4, 4,
						uvd_depth, 4, 1, xyzw2);

				cloud->points[j2].y = -xyzw2[0]/xyzw2[3];//y right+ (check)
				cloud->points[j2].z = -xyzw2[1]/xyzw2[3];//z up+
				cloud->points[j2].x = xyzw2[2]/xyzw2[3]; //x forward+
				unsigned char* rgba_ptr = (unsigned char*)&cloud->points[j2].rgba;
				// was bgr...
				cloud->points[j2].b =b;
				cloud->points[j2].r =r;
				cloud->points[j2].g =g;
				j2++;
			}
		}
		free(disparity_array);

	}else if(msg->depth.depth_data_format == KINECT_DEPTH_MSG_T_DEPTH_MM  ){
		/////////////////////////////////////////////////////////////////////
		// Openni Data
		// 1.2.2 unpack raw byte data into float values in mm

		// NB: no depth return is given 0 range - and becomes 0,0,0 here
		int npixels = msg->depth.width * msg->depth.height;
		const uint16_t* val = reinterpret_cast<const uint16_t*>( depth_data );
		
		cloud->width    =(int) (msg->depth.width/ (double) kinect_decimate) ;
		cloud->height   =(int) (msg->depth.height/ (double) kinect_decimate);
		//printf("Width=%d ,height=%d\n",cloud->width,cloud->height);
		cloud->is_dense = false;
		cloud->points.resize (cloud->width * cloud->height);
		double xyzw[4];
		int j=0;
		int j2=0;
		printf("Successfully log data\t");
		for(int v=0; v<cloud->height*kinect_decimate; v=v+ kinect_decimate) { // t2b self->height 480   //msg->depth.height
			for(int u=0; u<cloud->width*kinect_decimate; u=u+kinect_decimate ) {  //l2r self->width 640//msg->depth.width
				uint8_t r = rgb_data[v*msg->depth.width*3 + u*3 + 0];
				uint8_t g = rgb_data[v*msg->depth.width*3 + u*3 + 1];
				uint8_t b = rgb_data[v*msg->depth.width*3 + u*3 + 2];
				

				double constant = 1.0f / kcal->intrinsics_rgb.fx ;
				double disparity_d = val[v*msg->depth.width+u]  / 1000.0; // convert to m
				cloud->points[j2].y =  - (((double) u)- 319.50)*disparity_d*constant; //y right+ (check)
				cloud->points[j2].z = - (((double) v)- 239.50)*disparity_d*constant;  //z up+
				cloud->points[j2].x = disparity_d;  //x forward+
				if (disparity_d==0){ // place null points at negative range... arbitarty decision
					double disparity_d = -0.1; // convert to m
					cloud->points[j2].y =  - (((double) u)- 319.50)*disparity_d*constant; //y right+ (check)
					cloud->points[j2].z =  -(((double) v)- 239.50)*disparity_d*constant;  //z up+
					cloud->points[j2].x = disparity_d;  //x forward+

				}
				//printf("point_x=%f\tpoint_y=%f\tpoint_z=%f\n",cloud->points[j2].x,cloud->points[j2].y,cloud->points[j2].z);
				cloud->points[j2].b =b;
				cloud->points[j2].r =r;
				cloud->points[j2].g =g;
				j2++;
			}
		}

	}

	if(msg->depth.compression != KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
		
		free(uncompress_buffer); // memory leak bug fixed
	}
}

void unpack_kinect_frame_depth_vis(const kinect_frame_msg_t *msg,
		uint8_t* depth_img){

	int width = 640;
	int height = 480;
	int npixels = width*height;

	uint8_t* depth_uncompress_buffer = (uint8_t*)malloc(npixels*sizeof(uint16_t));
	uint16_t t_gamma[DEPTH_VAL];

	int i;
	for (i=0; i<DEPTH_VAL; i++) {
		float v = i/(float)DEPTH_VAL;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}

	if(msg->depth.depth_data_nbytes) {
		int i;
		const uint16_t* depth = NULL;
		if(msg->depth.compression == KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
			depth = (uint16_t*) msg->depth.depth_data;
		} else if (msg->depth.compression == KINECT_DEPTH_MSG_T_COMPRESSION_ZLIB) {
			unsigned long dlen = msg->depth.uncompressed_size;
			uncompress(depth_uncompress_buffer, &dlen, msg->depth.depth_data, msg->depth.depth_data_nbytes);
			depth = (uint16_t*) depth_uncompress_buffer;
			//printf("compressed size=%i\n", msg->depth.depth_data_nbytes);
			//printf("uncompressed size=%i\n", dlen);
		}

		int npixels = width * height;
		for (i=0; i<npixels; i++) {
			//printf("%i: %04X\n",  i, depth[i]);


//			int max = 2048;
//			int min = 800;
//			int p = (int)(255 * (depth[i] - min) / (max - min));
//			depth_img[i*3 + 0] = p;
//			depth_img[i*3 + 1] = p;
//			depth_img[i*3 + 2] = p;

			if ( depth[i] >= DEPTH_VAL ) {
				depth_img[3*i+0] = 0;
				depth_img[3*i+1] = 0;
				depth_img[3*i+2] = 0;
				continue;
			}
			int pval = t_gamma[depth[i]];
			int lb = pval & 0xff;
			switch (pval>>8) {
			case 0:
				depth_img[3*i+0] = 255;
				depth_img[3*i+1] = 255-lb;
				depth_img[3*i+2] = 255-lb;
				break;
			case 1:
				depth_img[3*i+0] = 255;
				depth_img[3*i+1] = lb;
				depth_img[3*i+2] = 0;
				break;
			case 2:
				depth_img[3*i+0] = 255-lb;
				depth_img[3*i+1] = 255;
				depth_img[3*i+2] = 0;
				break;
			case 3:
				depth_img[3*i+0] = 0;
				depth_img[3*i+1] = 255;
				depth_img[3*i+2] = lb;
				break;
			case 4:
				depth_img[3*i+0] = 0;
				depth_img[3*i+1] = 255-lb;
				depth_img[3*i+2] = 255;
				break;
			case 5:
				depth_img[3*i+0] = 0;
				depth_img[3*i+1] = 0;
				depth_img[3*i+2] = 255-lb;
				break;
			default:
				depth_img[3*i+0] = 0;
				depth_img[3*i+1] = 0;
				depth_img[3*i+2] = 0;
				break;
			}

		}
	}

	free(depth_uncompress_buffer);
}

}
