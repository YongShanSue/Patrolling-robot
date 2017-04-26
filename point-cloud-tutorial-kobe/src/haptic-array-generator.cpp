/*
 * haptic-array-generator.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: drc
 */

#include "haptic-array-generator.h"

using namespace std;
using namespace cv;

namespace rv {

int get_range_value(float v_in){
	if(v_in == 0){
		return 0;
	}
	int HAPTIC_MAX	= 255;
	int v = (int)(v_in * (float)HAPTIC_MAX / 6.0);
	if (v < 1) {
		v = 1;
	}else if (v > HAPTIC_MAX - 1) {
		v = HAPTIC_MAX - 1;
	}
	return v;
}

int get_motor_range_level(float v_in){
	if(v_in > 4.0){
		return 0;
	}else if(v_in > 2.0){
		return 1;
	}else if(v_in > 1.0){
		return 2;
	}else if(v_in > 0.5){
		return 3;
	}else if(v_in >= 0.0){
		return 4;
	}
}

std::vector<int> get_haptic_random(int n){

	std::vector<int> h_array;

	/* initialize random seed: */
	srand (time(NULL));

	for(int i = 0; i < n; i++){
		/* generate secret number between 1 and 10: */
		int r = rand() % 255 + 1;
		h_array.push_back(r);
	}

	return h_array;
}

// get the smallest except 0
void get_haptic_range_blocks(
		cv::Mat im_dist, std::vector<cv::Rect> blocks,
		std::vector<float> & ranges,
		std::vector<float> & valid_pt_ratios){

	// access range data
	for(int i = 0; i < blocks.size(); i++){

		float gd_pts = 0;
		//minimum
		float d = 99;
		for(int j = 0; j < blocks[i].width; j++){
			for(int k = 0; k < blocks[i].height; k++){
				float val = im_dist(blocks[i]).at<float>(k, j);
				if(val >= 0.3 && val < d){
					d = val;
				}
				if(val >= 0.3){
					gd_pts += 1;
				}
			}
		}

		valid_pt_ratios.push_back(
				gd_pts / (float)(blocks[i].width * blocks[i].height));

//		// access average
//		cv::Scalar d_s = mean(im_dist(blocks[i]));
//		float d = d_s[0];

		// single point access
//		cv::Point c = cv::Point(
//				blocks[i].x + blocks[i].width / 2,
//				blocks[i].y + blocks[i].height / 2
//		);
//		float d = im_dist.at<float>(c);

		// all pixels are < 0.3 -> set to 0.2
		if(d == 99){
			d = 0.2;
		}

		ranges.push_back(d);
	}

}

std::vector<int> get_haptic_range_grids(
		cv::Mat im_h, cv::Mat im_dist, cv::Mat im_z, int decimate){

	int HAPTIC_MAX	= 255;

	vector<int> haptic_array;

	vector<int> grids;
	grids.push_back(5);
	std::vector<cv::Rect> blocks = rv::get_haptic_range_grids_bboxes(grids,
			im_h.rows, im_h.cols, 16 / decimate);

	// TODO: need to check blocks dimension

	// access range data
	for(int i = 0; i < blocks.size(); i++){
		cv::Point c = cv::Point(
				blocks[i].x + blocks[i].width / 2,
				blocks[i].y + blocks[i].height / 2
		);

		// access average
		cv:Scalar h_s = mean( im_h(blocks[i]) );
		float h = h_s[0];
		cv::Scalar z_s = mean(im_z(blocks[i]));
		float z = z_s[0];
		cv::Scalar d_s = mean(im_dist(blocks[i]));
		float d = d_s[0];

		// single point access
//		float h = (float)im_h.at<uchar>(c);
//		float z = im_z.at<float>(c);
//		float d = im_dist.at<float>(c);

		int v = -1;
		if(h == 255.0 && z < 0.2){
			v = HAPTIC_MAX;
			//cout << "ground plane" << endl;
		}else{
			//v = rv::get_range_value(d);
			v = rv::get_motor_range_level(d);
		}
		haptic_array.push_back(v);
	}

	return haptic_array;
}

std::vector<cv::Rect> get_haptic_range_grids_bboxes(vector<int> grids,
		int rows, int cols, int s){
	// 320 x 240
	std::vector<cv::Rect> blocks;

	int grid_y = rows / (grids.size()+1);
	for(int i = 0; i < grids.size(); i++){
		int grid_x = cols / (grids[i] + 1);
		for(int j = 0; j < grids[i]; j++){
			Rect r = Rect(grid_x * (j + 1), grid_y * (i + 1), s, s);
			blocks.push_back(r);
		}
	}

	return blocks;
}

std::vector<int> get_haptic_polar_grids(cv::Mat im_g, int n, float patch_density){

	std::vector<int> h_array;

	// origin
	cv::Point pt_o = cv::Point(im_g.cols / 2, im_g.rows);

	std::vector<cv::Point> grid_pts = get_haptic_polar_grids_pts(im_g, n,
			patch_density);
	//cout << grid_pts.size() << endl;

	for(int i = 0; i < grid_pts.size(); i++){

		cv::Point pt_s = grid_pts[i];

		float dist = cv::norm(Mat(pt_o), Mat(pt_s));
		dist = dist / 300 * 6;

		//
		//int v = rv::get_range_value(dist);
		int v = rv::get_motor_range_level(dist);
		h_array.push_back(v);
	}

	return h_array;
}

void get_occupancy_grid_polar_pts(cv::Mat im_occupancy,
		std::vector<cv::Point> & dense_ground_pts,
		std::vector<cv::Point> & hit_pts,
		std::vector<int> & is_hits,
		std::vector<int> & enter_dense_zones,
		int n, float patch_density){

	// origin
	cv::Point pt_o = cv::Point(im_occupancy.cols / 2, im_occupancy.rows);

	// find the first non-zero row
	float fw_s = 100; // fw of "blind spot"
	float fw_step = 3;
	int max_step = 80; // fw_step * max_step = 300

	// theta
	float theta_s = 120;
	float theta_step = 60 / (n + 1);
//	float theta_s = 118;
//	float theta_step = 55 / (n);

	// for each direction
	for(int i = 0; i < n; i++){

		int enter_dense_zone = 0;
		cv::Point dense_ground_pt = pt_o;
		cv::Point hit_pt = pt_o;
		int is_hit = 0;

		float theta = theta_s - theta_step * (i + 1);
//		float theta = theta_s - theta_step * (i);
//		cout << "Direction: " << i << endl;

		for(int j = 0; j < max_step; j++){
			float r = 10 + fw_step * j;
			float x = r * cos(theta / 180.0 * M_PI);
			float y = r * sin(theta / 180.0 * M_PI);
			//cout << "theta, x, y = " << theta << ", " << x << ", " << y << endl;

			cv::Point pt_s = cv::Point(pt_o.x + x, pt_o.y - y);

			// access a patch,
			// there should
			// and get the average
			Rect r_patch = Rect(pt_s.x - 3, pt_s.y - 3, 6, 6);
			cv:Scalar r_s = sum(sum(im_occupancy(r_patch)));
			float r_sum = r_s[0];
			float r_ratio = r_sum / (float)(6 * 6 * 255);

			// enter dense zone
			if(enter_dense_zone == 0 && r_ratio > patch_density){
//				cout << "enter dense zone" << endl;
				enter_dense_zone = 1;
			}
			// check out of dense zone
			if(enter_dense_zone == 1 && r_ratio < patch_density){
//				cout << "out of dense zone" << endl;
				enter_dense_zone ++;
				dense_ground_pt = pt_s;
			}

			// check obstacle
			if((int)r_sum % 255 != 0){
				hit_pt = pt_s;
				is_hit = 1;
				// if still in dense zone
				if(enter_dense_zone == 1){
					dense_ground_pt = pt_s;
					enter_dense_zone = 3;
				}
//				cout << "hit obstacle" << endl;
				break;
			}else{

			}


			if(j == max_step - 1){
				hit_pt = pt_s;
				is_hit = 0;
				break;
			}
		}
		hit_pts.push_back(hit_pt);
		is_hits.push_back(is_hit);
		dense_ground_pts.push_back(dense_ground_pt);
		enter_dense_zones.push_back(enter_dense_zone);
	}

}

std::vector<cv::Point> get_haptic_polar_grids_pts(cv::Mat im_g, int n,
		float patch_density){

	std::vector<cv::Point> grid_pts;

	// origin
	cv::Point pt_o = cv::Point(im_g.cols / 2, im_g.rows);

	// find the first non-zero row
	float fw_s = 100; // fw of "blind spot"
	float fw_step = 10;
	int max_step = 28; // fw_step * max_step = 300

	// theta
	float theta_s = 120;
	float theta_step = 60 / (n + 1);

	for(int i = 0; i < n; i++){
		float theta = theta_s - theta_step * (i + 1);
		for(int j = 0; j < max_step; j++){
			float r = 10 + fw_step * j;
			float x = r * cos(theta / 180.0 * M_PI);
			float y = r * sin(theta / 180.0 * M_PI);
			//cout << "theta, x, y = " << theta << ", " << x << ", " << y << endl;

			cv::Point pt_s = cv::Point(pt_o.x + x, pt_o.y - y);

			// access a patch,
			// there should
			// and get the average
			Rect r_patch = Rect(pt_s.x - 3, pt_s.y - 3, 6, 6);
			cv:Scalar r_s = sum(sum(im_g(r_patch)));
			float r_sum = r_s[0];

			// check obstacle
			if((int)r_sum % 255 != 0){
				grid_pts.push_back(pt_s);
				break;
			}

			// when out of blind spot, check density
			if(r > fw_s && r_sum / (float)(6 * 6 * 255) < patch_density){
				grid_pts.push_back(pt_s);
				break;
			}

			if(j == max_step - 1){
				grid_pts.push_back(pt_s);
				break;
			}
		}
	}

	return grid_pts;
}

void get_haptic_polar_grids_pts_3d(cv::Mat im_g, int n,
		std::vector<cv::Point> &grid_pts,
		std::vector<Eigen::Vector3d> &grid_pts_3d,
		float patch_density){

	// origin
	cv::Point pt_o = cv::Point(im_g.cols / 2, im_g.rows);

	// find the first non-zero row
	float fw_s = 100; // fw of "blind spot"
	float fw_step = 10;
	int max_step = 28; // fw_step * max_step = 300

	// theta
	float theta_s = 120;
	float theta_step = 60 / (n + 1);

	for(int i = 0; i < n; i++){
		float theta = theta_s - theta_step * (i + 1);
		for(int j = 0; j < max_step; j++){
			float r = 10 + fw_step * j;
			float x = r * cos(theta / 180.0 * M_PI);
			float y = r * sin(theta / 180.0 * M_PI);
			//cout << "theta, x, y = " << theta << ", " << x << ", " << y << endl;

			cv::Point pt_s = cv::Point(pt_o.x + x, pt_o.y - y);

			// access a patch, and get the average
			Rect r_patch = Rect(pt_s.x - 3, pt_s.y - 3, 6, 6);
			cv:Scalar r_s = sum(sum(im_g(r_patch)));
			float r_sum = r_s[0];

			// check obstacle
			if((int)r_sum % 255 != 0){
				grid_pts.push_back(pt_s);
				break;
			}

			// when out of blind spot, check density
			if(r > fw_s && r_sum / (float)(6 * 6 * 255) < patch_density){
				grid_pts.push_back(pt_s);
				break;
			}

			if(j == max_step - 1){
				grid_pts.push_back(pt_s);
				break;
			}
		}
	}

}

std::vector<cv::Rect> get_haptic_array_blocks(cv::Mat im, int rows, int cols){

	std::vector<cv::Rect> blocks;

	int grid_y = im.rows / rows;
	int grid_x = im.cols / cols;

	for(int i = 0; i < rows; i++){
		for(int j = 0; j < cols; j++){
			int x = j * grid_x;
			int y = i * grid_y;
			cv::Rect r = Rect(x, y, grid_x, grid_y);
			r = r & Rect(0, 0, im.cols, im.rows);

			blocks.push_back(r);
		}
	}

	return blocks;
}

std::vector<cv::Rect> get_haptic_array_blocks(cv::Mat im, int h_fov){

}

std::vector<int> get_haptic_depth_grid(cv::Mat depth_map, int rows, int cols){

	std::vector<int> h_array;

	vector<Rect> blocks = get_haptic_array_blocks(depth_map, rows, cols);
	for(int i = 0; i < blocks.size(); i++){
		Mat im_b = depth_map(blocks[i]);


	}

	return h_array;
}

cv::Point2f get_end_pt_2d(Eigen::Vector3d end_pt_3d,
		float default_pitch, float default_height){

	// TODO: this might be done by a camera matrix?

	float WIDTH_PX = 640;
	float HEIGHT_PX = 480;
	float H_FOV = 60;
	float V_FOV = 40;

	end_pt_3d += Eigen::Vector3d(0, 0, -default_height);

	//cout << end_pt_3d << endl;
	float theta_h = atan(end_pt_3d[1]/end_pt_3d[0]) * 180 / M_PI;
	float theta_v = atan(end_pt_3d[2]/end_pt_3d[0]) * 180 / M_PI + default_pitch;

	// check in FOV

	//
	float x_px = (WIDTH_PX / 2) - theta_h / (H_FOV / 2) * (WIDTH_PX / 2);
	float y_px = (HEIGHT_PX / 2) - theta_v / (V_FOV / 2) * (HEIGHT_PX / 2);

	cout << theta_h << " x " << theta_v << endl;
	cout << x_px << " x " << y_px << endl;

	return cv::Point2f(x_px, y_px);
}

}


