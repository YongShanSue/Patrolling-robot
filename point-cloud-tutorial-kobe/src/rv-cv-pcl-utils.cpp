/**
 * @file util.h
 * @brief Monocular SLAM with April tags and iSAM
 * @author: Michael Kaess
 *
 */
#include "rv-cv-pcl-utils.h"

using namespace Eigen;
using namespace std;
using namespace cv;

namespace rv {

Eigen::Vector3i get_xyz_to_grid(Eigen::Vector3f p,
		int m_x, int m_y, int m_z,
		int g_x, int g_y){

	// right hand coordinates
	// x+: forward, y+: left, z+: up
	int x_i = -1; // the x index on the grid
	int y_i = -1; // the y index on the grid
	int z_i = -1; // the z index on the grid

	// p should be inside forward m_y and left/right 0.5 m_x, and lower than m_z
	if(p[1] < m_x / 2 && p[1] > -m_x / 2 &&
			p[0] > 0 && p[0] < m_y &&
			p[2] > -m_z / 2 && p[2] < m_z / 2){

		x_i = g_x - (p[1] + m_x / 2) * (g_x / m_x);

		y_i = g_y - p[0] * (g_y / m_y);

		if(p[2] < -0.2 || p[2] >= 1.3){
			z_i = 0;
		}else if(p[2] < 0.2){
			z_i = 1;
		}else if(p[2] < 0.6){
			z_i = 2;
		}else if(p[2] < 1.3){
			z_i = 3;
		}
	}

	Eigen::Vector3i grid_xyz;
	grid_xyz << x_i, y_i, z_i;

	return grid_xyz;
}

void get_surface_masks(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Mat &im_n){

}

int get_surface_normals(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::Normal>::Ptr normals,
		cv::Mat &im_n, cv::Mat &im_v, cv::Mat &im_h,
		cv::Mat &im_dist, cv::Mat &im_z){

	int gd_pts = 0;

//	// normal estimation
//	pcl::PointCloud<pcl::Normal>::Ptr normals;
//	normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
//	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
//	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
//	ne.setMaxDepthChangeFactor(scale);
//	ne.setNormalSmoothingSize(10.0f);
//	ne.setInputCloud(cloud);
//	ne.compute(*normals);

	Eigen::Vector3d vec_up(0, 0, 1); // upward
	Eigen::Vector3d vec_fw(1, 0, 0); // forward
	for(int i = 0; i < cloud->width; i++){
		for(int j = 0; j < cloud->height; j++){

			// check null point
			if(isnan(normals->at(i, j).normal_x)){
				continue;
			}

			// dist
			Eigen::Vector3d vec_o_2d(
					cloud->at(i, j).x,
					cloud->at(i, j).y,
					0);
			double dist = vec_o_2d.norm();
			if(dist > 6){
				continue;
			}

			im_dist.at<float>(j, i) = (float)dist;

			// height
			im_z.at<float>(j, i) = cloud->at(i, j).z;

			// drop points too far away when calcuate normals
			if(dist > 6){
				continue;
			}

			// surface normal vector
			Eigen::Vector3d vec_n(
					normals->at(i, j).normal_x,
					normals->at(i, j).normal_y,
					normals->at(i, j).normal_z);
			im_n.at<Vec3f>(j, i)[0] = vec_n[0];
			im_n.at<Vec3f>(j, i)[1] = vec_n[1];
			im_n.at<Vec3f>(j, i)[2] = vec_n[2];


			// find horizontal/vertical surfaces
			int is_h = -1;
			int is_v = -1;
			double cos_up_n = vec_up.dot(vec_n);
			double cos_fw_n = vec_fw.dot(vec_n);

			// normal should point up, angle < 35 degree
			if(cos_up_n > 0.8){
				im_h.at<uchar>(j, i) = 255;
				is_h = 1;
				gd_pts++;

			}
			if(abs(cos_up_n) < 0.2 ){//&& cos_fw_n < -0.2 ){
				im_v.at<uchar>(j, i) = 255;
				is_v = 1;
				gd_pts++;
			}

		}
	}

	return gd_pts;
}

int get_ground_surface(
		cv::Mat im_v, cv::Mat im_h, cv::Mat im_z, cv::Mat im_dist, cv::Mat im_n,
		cv::Mat &im_stixel, Eigen::Vector3f &est_nor){

	int stixel_col = 5;
	int pad_size = 10;

	int first_cell_px = im_stixel.rows - pad_size - stixel_col;

	Eigen::Vector3f g_n_sum_p;
	float g_n_count_p = 0;

	// let's just assume that ground can be seen first
	// search from the bottom-up in vertical direction
	for(int i = pad_size; i < im_stixel.cols + pad_size; i = i + stixel_col){
		int prev_label = 1;
		float prev_z = 0;
		float prev_dist = 0;

		for(int j = first_cell_px; j >= pad_size; j = j - stixel_col){
			Rect rect = Rect(i, j, stixel_col, stixel_col);
			Rect r_in = rect & Rect(0, 0, im_stixel.cols, im_stixel.rows);

			if(r_in.area() != rect.area()){
				continue;
			}

			// check im_h sum
			int h_sum = countNonZero(im_h(rect));
			float h_density = (float)h_sum / (float)rect.area();

			int v_sum = countNonZero(im_v(rect));
			float v_density = (float)v_sum / (float)rect.area();

			float z = im_z.at<float>(j, i);
			float delta_z = abs(z - prev_z);
			prev_z = z;

			float dist = im_dist.at<float>(j, i);
			float delta_dist = abs(dist - prev_dist);
			prev_dist = dist;

			Eigen::Vector3f vec_nor(
					im_n.at<Vec3f>(j, i)[0],
					im_n.at<Vec3f>(j, i)[1],
					im_n.at<Vec3f>(j, i)[2]);

//			if(vec_nor.norm() != 1){
//				cout << "weird normal" << endl;
//				continue;
//			}
//
			for(int k = 0; k < 3; k++){
				if(abs(vec_nor[k]) > 100){
					cout << "weird normal2" << endl;
					continue;
				}
			}

			// first cell must be ground/horizontal
			if(j == first_cell_px && h_density < 0.5){
				prev_label = 0;
			}

			// this column is done
			if(prev_label == 0){

			}else if(prev_label == 1){
				if(h_density > 0.5){
					im_stixel(rect).setTo(1);

					g_n_sum_p += vec_nor;
					g_n_count_p ++;

				}else if(v_density > 0.5){
					im_stixel(rect).setTo(2);

					prev_label = 2;
				}else{
					// this column is done
					prev_label = 0;
				}
			}else if(prev_label == 2){
				if(v_density > 0.5){
					im_stixel(rect).setTo(2);
				}else{
					// this column is done
					prev_label = 0;
				}
			}

//
//			cout << "h_density:\t" << h_density << endl;
//			cout << "v_density:\t" << v_density << endl;
//			cout << "delta_z:\t" << delta_z << endl;
//			cout << "delta_dist:\t" << delta_dist << endl;
//			Eigen::Vector3f vec_fw(1, 0, 0); // forward
//
//			cout << vec_fw.dot(vec_nor) << endl;
//			cout << endl;
//
//			Mat im_vis = im_h.clone();
//			cv::rectangle(im_vis, rect.tl(), rect.br(), Scalar(255, 0, 0), 1);
//
//			cv::namedWindow("stixel", 0);
//			cv::imshow("stixel", im_vis);
//			cv::waitKey(0);


		}
	}

	if(g_n_count_p < 20){
		return 0;
	}

	// TODO: a weird overflow?
	//cout << "There are " << g_n_count_p << " ground patches\t";
	//cout << "Total: " << g_n_sum_p << endl;
	for(int k = 0; k < 3; k++){
		//cout << g_n_sum_p[k] << ", ";
		if(g_n_sum_p[k] > 1000){
			cout << "weird: " << k << " " << g_n_sum_p[k] << endl;
		}
		est_nor[k] = g_n_sum_p[k] / g_n_count_p;
	}
	cout << endl;
	//est_nor = g_n_sum_p / g_n_count_p;
	// clear floating point

	//cout << est_nor << endl;
	est_nor = est_nor / est_nor.norm();

	if(est_nor[0] == 0){
		cout << g_n_sum_p[0] << ", " << g_n_sum_p[1] << ", " << g_n_sum_p[2] << endl;
		cout << est_nor.norm() << endl;
	}
	return 1;
	//
}

int get_flat_surface(cv::Rect bbox, cv::Mat im_n, cv::Mat im_v){

	int step = bbox.width / 10;

	Eigen::Vector3f g_n_sum_p;
	float g_n_count_p = 0;

	for(int i = 0; i < bbox.width; i+= step){
		for(int j = 0; j < bbox.height; j+= step){

			int is_v = im_v.at<uchar>(j, i);
			if(is_v == 0){
				continue;
			}

			Eigen::Vector3f vec_nor(
					im_n.at<Vec3f>(j, i)[0],
					im_n.at<Vec3f>(j, i)[1],
					im_n.at<Vec3f>(j, i)[2]);

			g_n_sum_p += vec_nor;
			g_n_count_p ++;
		}
	}

	Eigen::Vector3f est_nor;
	for(int k = 0; k < 3; k++){
		//cout << g_n_sum_p[k] << ", ";
		if(g_n_sum_p[k] > 1000){
			cout << "weird: " << k << " " << g_n_sum_p[k] << endl;
		}
		est_nor[k] = g_n_sum_p[k] / g_n_count_p;
	}
	est_nor = est_nor / est_nor.norm();
	cout << est_nor << endl;

	// sec loop to get the average cosine, flat should be close to 1
	float sum_cos = 0;
	for(int i = 0; i < bbox.width; i+= step){
		for(int j = 0; j < bbox.height; j+= step){

			int is_v = im_v.at<uchar>(j, i);
			if(is_v == 0){
				continue;
			}

			Eigen::Vector3f vec_nor(
					im_n.at<Vec3f>(j, i)[0],
					im_n.at<Vec3f>(j, i)[1],
					im_n.at<Vec3f>(j, i)[2]);

			// |a| and |b| are both 1
			float nor_cos = est_nor.dot(vec_nor);
			sum_cos += (nor_cos);
		}
	}
	cout << "cos: " << sum_cos / g_n_count_p << endl;

	return 0;
}

void get_ground_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat im_ground,
		float & height_avg, float & height_std, vector<float> & block_heights){

	float height_sum = 0;
	float height_count = 0;

	vector<float> vec_height_sum;
	vector<float> vec_height_count;
	for(int i = 0; i < 5; i++){
		vec_height_sum.push_back(0);
		vec_height_count.push_back(0);
	}
	int block_width = cloud->width / 5;

	for(int i = 0; i < cloud->width; i++){
		for(int j = 0; j < cloud->height; j++){
			if(im_ground.at<uchar>(j, i) > 0){
				height_sum += cloud->at(i, j).z;
				height_count ++;

				int block_idx = (int)((float)i / (float)block_width);
				vec_height_sum[block_idx] += cloud->at(i, j).z;
				vec_height_count[block_idx] ++;
			}
		}
	}

	height_avg = height_sum / height_count;

	float sq_sum = 0;
	for(int i = 0; i < cloud->width; i++){
		for(int j = 0; j < cloud->height; j++){
			if(im_ground.at<uchar>(j, i) > 0){
				sq_sum += ((cloud->at(i, j).z - height_avg) * (cloud->at(i, j).z - height_avg));
			}

		}
	}
	height_std = sqrt(sq_sum / height_count);

	for(int i = 0; i < 5; i++){
		float block_height;
		if(vec_height_count[i] == 0){
			block_height = -1.0;
		}else{
			block_height = vec_height_sum[i] / vec_height_count[i];
		}
		//cout << "block height: " << block_height << endl;
		block_heights[i] = block_height;
	}

	//cout << "end get height" << endl;
}

void get_occupancy_grid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Mat im_v, cv::Mat im_h, cv::Mat im_ground,
		cv::Mat &im_occupancy){

	for(int i = 0; i < cloud->width; i++){
		for(int j = 0; j < cloud->height; j++){

			int is_v = im_v.at<uchar>(j, i);
			int is_h = im_h.at<uchar>(j, i);
			int is_ground = im_ground.at<uchar>(j, i);

			if(is_ground > 0){
				continue;
			}

			// accumulate all non-ground pts
			if(is_v > 0 || is_h > 0){
				// convert coordinate
				Eigen::Vector3f p(
						cloud->at(i, j).x,
						cloud->at(i, j).y,
						cloud->at(i, j).z);
				Eigen::Vector3i grid_xyz = get_xyz_to_grid(p);

				// outside 6x6 box
				if(grid_xyz[0] == -1){
					continue;
				}

				// clear horizontal pts about ground height
				if(is_h > 0 && grid_xyz[2] == 1){
					continue;
				}

				im_occupancy.at<uchar>(grid_xyz[1], grid_xyz[0]) += 1;


			}
		}
	}

}

void get_occupancy_grid_view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Mat im_v, cv::Mat im_h,
		cv::Mat &im_m, cv::Mat &im_g, cv::Mat &im_g1){

	for(int i = 0; i < cloud->width; i++){
		for(int j = 0; j < cloud->height; j++){

			int is_v = im_v.at<uchar>(j, i);
			int is_h = im_h.at<uchar>(j, i);

			// convert coordinate
			Eigen::Vector3f p(
					cloud->at(i, j).x,
					cloud->at(i, j).y,
					cloud->at(i, j).z);
			Eigen::Vector3i grid_xyz = get_xyz_to_grid(p);

			// outside 6x6 box
			if(grid_xyz[0] == -1){
				continue;
			}

			if(is_h > 0){
				if(grid_xyz[2] == 1){
					// ground surface
					im_g.at<uchar>(grid_xyz[1], grid_xyz[0]) = 255;
					im_m.at<uchar>(j, i) = 255;
				}
				if(grid_xyz[2] == 2){
					// knee height
					im_g.at<uchar>(grid_xyz[1], grid_xyz[0]) = 2;
					im_m.at<uchar>(j, i) = 2;
				}else if(grid_xyz[2] == 3){
					// waist height
					im_g.at<uchar>(grid_xyz[1], grid_xyz[0]) = 3;
					im_m.at<uchar>(j, i) = 3;
				}else if(grid_xyz[2] == 0){
					// obstacle
					im_g.at<uchar>(grid_xyz[1], grid_xyz[0]) = 1;
					im_m.at<uchar>(j, i) = 1;
				}
				im_g1.at<Vec4b>(grid_xyz[1], grid_xyz[0])[grid_xyz[2]] = 255;
			}else if(is_v > 0){
				im_g.at<uchar>(grid_xyz[1], grid_xyz[0]) = 1;
				im_m.at<uchar>(j, i) = 1;
				im_g1.at<Vec4b>(grid_xyz[1], grid_xyz[0])[grid_xyz[2]] = 128;
			}

		}
	}
}

void get_nor_height_vec(cv::Rect box,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Mat im_v, cv::Mat im_h, cv::Mat im_ground,
		cv::Mat & nor_height_vec,
		float height_min, float height_step, int height_max_idx){

	// create a vector to hold the data
	vector<int> fea_vec;
	for(int i = 0; i < nor_height_vec.cols; i++){
		fea_vec.push_back(0);
	}

	for(int i = box.x; i < box.x + box.width-1; i++){
		for(int j = box.y; j < box.y + box.height-1; j++){

			int is_v = im_v.at<uchar>(j, i);
			int is_h = im_h.at<uchar>(j, i);
			int is_ground = im_ground.at<uchar>(j, i);

			if(is_ground > 0){
				continue;
			}

			// accumulate all non-ground pts
			if(is_v > 0 || is_h > 0){
				Eigen::Vector3f p(
						cloud->at(i, j).x,
						cloud->at(i, j).y,
						cloud->at(i, j).z);

				// neglect height lower than min
				if (p[2] <= height_min){
					continue;
				}

				// (p[2]-height_min) must >= 0
				int height_idx = int((p[2]-height_min) / height_step);
				if (height_idx >= height_max_idx){
					height_idx = height_max_idx -1;
				}

				// vertical first then horizontal
				int vh_idx = 0;
				if(is_h > 0){
					vh_idx = 1;
				}
				int fea_vec_idx = height_idx + vh_idx * height_max_idx;

				if(fea_vec.size() <= fea_vec_idx){
					cout << p[2] << endl;
					cout << height_idx << endl;
					cout << vh_idx << endl;
					cout << height_max_idx << endl;
					cout << "size is wried: " << fea_vec.size() << " " << fea_vec_idx << endl;
				}
				int current_sum = fea_vec[fea_vec_idx];

				if(current_sum < 1000){
					fea_vec[fea_vec_idx] = current_sum + 1;
				}

//				cout << height_idx + vh_idx * height_max_idx << "\t";
//				cout << nor_height_vec.at<float>(0, height_idx + vh_idx * height_max_idx) << endl;
				//fea_vec[fea_vec_idx] += 1;
				//nor_height_vec.at<float>(0, fea_vec_idx)++;

			}
		}
	}

	// remove ground
	// feature vector 36 dimension:
	// [v_-0.3, v_-0.2, v_-0.1, v_0, ..., v_1.5,	->	 0-17
	//  h_-0.3, h_-0.2, h_-0.1, h_0, ..., h_1.5]	->	18-35
	//                          g_idx
	//							21
	int ground_height_idx =  int((0-height_min) / height_step);
	int fea_vec_g_idx = ground_height_idx + 1 * height_max_idx;
	fea_vec[fea_vec_g_idx] = 0;
	fea_vec[fea_vec_g_idx-1] = 0;

	int max_count = 0;
	int h_max_count = 0;
	int h_sum = 0;
	int v_max_count = 0;
	int v_sum;
	int px_avg_count =  box.area() / (fea_vec.size()/2);

	for(int i = 0; i < fea_vec.size(); i++){
		//vertical
		if(i < fea_vec.size() / 2){
			if(fea_vec[i] > v_max_count){
				v_max_count = fea_vec[i];
			}
			v_sum += fea_vec[i];
		}else{	// horizontal
			if(fea_vec[i] > h_max_count){
				h_max_count = fea_vec[i];
			}
			h_sum += fea_vec[i];
		}
		if(fea_vec[i] > max_count){
			max_count = fea_vec[i];
		}
	}

	// normalization
	// make sure the activation is good enough:

	// vertical case
	for(int i = 0; i < fea_vec.size(); i++){

		int vh_max_count;
		if(i < fea_vec.size()/2){
			vh_max_count = v_max_count;
		}else{
			vh_max_count = h_max_count;
		}

//		// at least one should greater than px_avg_count
//		if(vh_max_count > px_avg_count){
			float activation = (float)fea_vec[i] / (float)px_avg_count;
			if(activation > 1){
				activation = 1;
			}
			nor_height_vec.at<float>(0, i) = activation;
//		}else{
//			nor_height_vec.at<float>(0, i) = 0;
//		}
	}

	// debug
	int DEBUG = 0;
	if (DEBUG == 0){
		return;
	}
	cout << "BBox Size: " << box.area() << "; each block: "
			<< px_avg_count << endl;

	cout << "vertical: total " << v_sum  << ", max: " << v_max_count << endl;
	cout << "horizontal: total " << h_sum  << ", max: " << h_max_count << endl;

	cout << "v: ";
	for(int i = 0; i < fea_vec.size(); i++){
		cout << std::setw(4) << fea_vec[i];
		if(i == fea_vec.size() / 2 -1){
			cout << endl;
			cout << "h: ";
		}
	}
	cout << endl;

	cout << "feature vector:" << endl;
	cout << "v: ";
	for(int i = 0; i < fea_vec.size(); i++){
		cout << std::fixed << setprecision(2) << nor_height_vec.at<float>(0, i) << " ";
		if(i == fea_vec.size() / 2 -1){
			cout << endl;
			cout << "h: ";
		}
	}
	cout << endl;
}

void get_relative_transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Mat im_ground, int decimate,
		double fx, double fy, double px, double py,
		Eigen::Matrix<double, 3, 4> &P){

	std::vector<cv::Point3f> objPts;
	std::vector<cv::Point2f> imgPts;

	// pick a few points on the im_ground
	int step = 5;
	int nPts = 0;
	for(int i = 0; i < im_ground.cols; i=i+step){
		for(int j = 0; j < im_ground.rows; j=j+step){

			int is_g = im_ground.at<uchar>(j, i);
			if(is_g > 0){
				cv::Point2f p2 = cv::Point2f(i * decimate, j * decimate);
				imgPts.push_back(p2);

				Eigen::Vector3f p(
						cloud->at(i, j).x,
						cloud->at(i, j).y,
						0);
				cv::Point3f p3 = cv::Point3f(p[0], p[1], p[2]);
				objPts.push_back(p3);

				nPts ++;
			}

		}
	}

	if(objPts.size() < 4){
		cout << "no ground surface!!" << endl;
		return;
	}else{
		//cout << "found " << objPts.size() << " pts" << endl;
	}

	cv::Mat rvec, tvec;
	cv::Matx33f cameraMatrix(
			fx, 0, px,
			0, fy, py,
			0,  0,  1);
	cv::Vec4f distParam(0,0,0,0); // all 0?
	cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
	cv::Matx33d r;
	cv::Rodrigues(rvec, r);
	Eigen::Matrix3d wRo;
	wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

	Eigen::Matrix4d T;
	T.topLeftCorner(3,3) = wRo;
	T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
	T.row(3) << 0,0,0,1;

	Eigen::Matrix<double, 3, 4> K;
	K <<
		   fx,	0,	px,	0,
			0, fy,	py,	0,
			0,	0,	 1,	0;
	P = K * T;

}

void get_relative_obj_img_corners(
		Eigen::Matrix<double, 3, 4> P,
		std::vector<Eigen::Vector3d> obj_corners,
		std::vector<cv::Point> &img_corners){

	Eigen::Vector3d x;
	for(int i = 0; i < obj_corners.size(); i++){

		Eigen::Vector4d v;
		v << obj_corners[i][0], obj_corners[i][1], obj_corners[i][2], 1;
		v.transpose();

		x = P * v;
		x = x / x(2);

		img_corners.push_back(cv::Point2f(x(0), x(1)));
	}
}

void get_cube_corners(Eigen::Vector3d tl_pt, Eigen::Vector3d cube_dim,
		vector<Eigen::Vector3d> & corners){
	double tl_dx = tl_pt[0];
	double tl_dy = tl_pt[1];
	double tl_dz = tl_pt[2];
	double sx = cube_dim[0];
	double sy = cube_dim[1];
	double sz = cube_dim[2];

	Eigen::Vector3d tl, tr, br, bl, tl2, tr2, br2, bl2;
	tl << tl_dx, tl_dy, tl_dz;
	tr = tl + Eigen::Vector3d(sx,  0, 0);
	br = tl + Eigen::Vector3d(sx,-sy, 0);
	bl = tl + Eigen::Vector3d( 0,-sy, 0);
	tl2 = tl + Eigen::Vector3d( 0,  0, sz);
	tr2 = tr + Eigen::Vector3d( 0,  0, sz);
	br2 = br + Eigen::Vector3d( 0,  0, sz);
	bl2 = bl + Eigen::Vector3d( 0,  0, sz);

	corners.push_back(tl);
	corners.push_back(tr);
	corners.push_back(br);
	corners.push_back(bl);
	corners.push_back(tl2);
	corners.push_back(tr2);
	corners.push_back(br2);
	corners.push_back(bl2);
}

void draw_cube_img_corners(cv::Mat &im_vis, std::vector<cv::Point> pts){
	// draw
	for(int i = 0; i < pts.size(); i++){
		cv::circle(im_vis, pts[i], 2, cv::Scalar(255,0,255), 2);
	}

	for(int i = 0; i < 3; i++){
		cv::line(im_vis,  pts[i], pts[i+1], cv::Scalar(255, 0,255, 0), 2);
		cv::line(im_vis,  pts[i+4], pts[i+4+1], cv::Scalar(255, 0,255, 0), 2);
	}
	cv::line(im_vis,  pts[0], pts[3], cv::Scalar(255, 0,255, 0), 2);
	cv::line(im_vis,  pts[4], pts[7], cv::Scalar(255, 0,255, 0), 2);

	for(int i = 0; i < 4; i++){
		cv::line(im_vis,  pts[i], pts[i+4], cv::Scalar(255, 0,255, 0), 2);
	}
}

//////////////////////////////////////////////////////////////////////

void get_shifted_point(cv::Point src_pt, cv::Rect search_rect,
		float offset_x_m, float offset_y_m,
		cv::Mat im_rgb_rect, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Point & p_shifted_2d, Eigen::Vector3d & p_shifted_3d){

	// get the src 3d point in cloud dimentions
	Eigen::Vector3d p_3d = get_cloud_3d_point(src_pt, im_rgb_rect, cloud);
	cv::Point p_2d = get_scaled_cloud_point(src_pt, im_rgb_rect, cloud);

	// find the rect for cloud
	Rect r = get_scaled_rect(search_rect, im_rgb_rect, cloud);

	// search within search_rect to find a point which is close to the offset point
	double min_dist_x = 99;
	double min_dist_y = 99;
	int final_x = 0;
	int final_y = 0;

	Mat dist_x_mx = Mat::zeros(r.height, r.width, CV_32F);

	for(int i = r.x; i < r.x + r.width; i++){
		for(int j = r.y; j < r.y + r.height; j++){
			Eigen::Vector3d pt = Eigen::Vector3d(
					cloud->at(i, j).x, cloud->at(i, j).y, cloud->at(i, j).z);

			// assume on vertical surface, get the delta in horizontal and vertical
			Eigen::Vector3d pt_h = Eigen::Vector3d(pt[0], pt[1], p_3d[2]);
			Eigen::Vector3d vec_h = pt_h - p_3d;
			float delta_x = vec_h.norm();
			float delta_y = abs(pt[2] - p_3d[2]);

			float dist_x, dist_y;
			// determine left or right
			if(i < p_2d.x){
				dist_x = abs(delta_x + offset_x_m);
			}else{
				dist_x = abs(delta_x - offset_x_m);
			}
			// debug
			//dist_x_mx.at<float>(j - r.y, i - r.x) = dist_x;

			// determine up or down
			if(j < p_2d.y){
				dist_y = abs(delta_y - offset_y_m);
			}
			//cout << "dist: " << dist_h << "," << dist_v << endl;

			if (dist_x < min_dist_x){
				min_dist_x = dist_x;
				final_x = i;

				p_shifted_2d = get_scaled_image_point(cv::Point(final_x, final_y), im_rgb_rect, cloud);
				p_shifted_3d = pt;
				//cout << "min dist: " << min_dist_x << ", " << min_dist_y << endl;
				//cout << "final x" << final_x << endl;
			}
			if (dist_y < min_dist_y){
				min_dist_y = dist_y;
				final_y = j;

				p_shifted_2d = get_scaled_image_point(cv::Point(final_x, final_y), im_rgb_rect, cloud);
				p_shifted_3d = pt;
				//cout << "min dist: " << min_dist_x << "," << min_dist_y << endl;

			}
		}
	}

	//cout << dist_x_mx << endl;

}

cv::Point get_scaled_image_point(cv::Point p,
		cv::Mat im_rgb_rect, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

	cv::Point pt;

	int scale = im_rgb_rect.cols / cloud->width;
	int y_offset = (cloud->height - im_rgb_rect.rows / scale) / 2 + 10;

	pt = cv::Point(p.x * 4, (p.y - y_offset) * 4);

	return pt;
}

cv::Point get_scaled_cloud_point(cv::Point p,
		cv::Mat im_rgb_rect, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

	// im_rgb_rect 1280 x 720 (16:9)
	// cloud 320 x 240 (4:3)
	int scale = im_rgb_rect.cols / cloud->width;
	// hack:
	int y_offset = (cloud->height - im_rgb_rect.rows / scale) / 2 + 10;

	int x = p.x / scale;
	int y = p.y / scale + y_offset;

	return cv::Point(x, y);
}

Eigen::Vector3d get_cloud_3d_point(cv::Point p,
		cv::Mat im_rgb_rect, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

	cv::Point p_scaled = get_scaled_cloud_point(p, im_rgb_rect, cloud);

	int x = p_scaled.x;
	int y = p_scaled.y;

	// TODO: check if this point is a null

	Eigen::Vector3d q;
	q << cloud->at(x, y).x, cloud->at(x, y).y, cloud->at(x, y).z;


	return q;
}

cv::Rect get_scaled_rect(cv::Rect r,
			cv::Mat im_rgb_rect, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

	cv::Point p_tl = get_scaled_cloud_point(r.tl(), im_rgb_rect, cloud);
	cv::Point p_br = get_scaled_cloud_point(r.br(), im_rgb_rect, cloud);

	int x = p_tl.x;
	int y = p_tl.y;
	int w = p_br.x - p_tl.x;
	int h = p_br.y - p_tl.y;

	return Rect(x, y, w, h);
}

int check_spatial_prior(cv::Mat im, cv::Rect r, vector<Eigen::Vector3d> &pts,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		cv::Mat im_vertical, cv::Mat im_dist,
		double phy_size_min, double phy_size_max){

	Scalar s = cv::mean(im_vertical(r));

	vector<cv::Point> cv_pts;
	cv::Point cv_p1 = r.tl();
	cv::Point cv_p2 = cv::Point(r.x + r.width, r.y);
	cv::Point cv_p3 = r.br();
	cv::Point cv_p4 = cv::Point(r.x, r.y + r.height);
	cv_pts.push_back(cv_p1);
	cv_pts.push_back(cv_p2);
	cv_pts.push_back(cv_p3);
	cv_pts.push_back(cv_p4);

	double min = 999;
	double max = -999;
	int has_3d_corners = 0;
	for(int i = 0; i < cv_pts.size(); i++){

		pcl::PointXYZRGB p = cloud->at(cv_pts[i].x, cv_pts[i].y);

		Eigen::Vector3d pt = Eigen::Vector3d(p.x, p.y, p.z);
		pts.push_back(pt); // for vis

		if(p.z < min){
			min = p.z;
		}
		if(p.z > max){
			max = p.z;
		}
	}

	double box_height = max - min;

	//
	if (s[0] > 128 & box_height > phy_size_min & box_height < phy_size_max){
		//cout << " avg surface: " << s[0] << ", " << box_height << " " << endl;
		return 1;
	}else{
		return 0;
	}


}




void crop_cloud(cv::Mat im, std::vector<cv::Rect> r,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_r){


}

Eigen::Matrix4d homographyToPose(double fx, double fy, double tag_size, Eigen::Matrix3d h) {


#if 0 // we recover camera based pose, then convert later to robot pose
	// flip the homography along the Y axis to align the
	// conventional image coordinate system (y=0 at the top) with
	// the conventional camera coordinate system (y=0 at the
	// bottom).
	Matrix3d F;
	F << 1,  0,  0,
			0, -1,  0,
			0,  0, -1;
	Matrix3d tmp = F * h;
	h = tmp;
#endif

	Matrix4d M = Matrix4d::Zero();
	M << h(0,0)/fx, h(0,1)/fx, 0, h(0,2)/fx,
			h(1,0)/fy, h(1,1)/fy, 0, h(1,2)/fy,
			h(2,0),    h(2,1),    0, h(2,2),
			0, 0, 0, 1;

	// Compute the scale. The columns of M should be made to be
	// unit vectors. This is over-determined, so we take the
	// geometric average.
	double scale = sqrt(M.col(0).norm()*M.col(1).norm());
	M *= 1.0/scale;

	// recover sign of scale factor by noting that observations must occur in front of the camera.
	if (M(2,3) <= 0) M *= -1.0; // kaess: sign swapped from original code (also see disabled F above)

	// The bottom row should always be [0 0 0 1].  We reset the
	// first three elements, even though they must be zero, in
	// order to make sure that they are +0. (We could have -0 due
	// to the sign flip above. This is theoretically harmless but
	// annoying in practice.)
	M.row(3).setZero();
	M(3,3) = 1;

	// recover third rotation vector by crossproduct of the other two rotation vectors.
	Vector3d a = M.col(0).topRows(3);
	Vector3d b = M.col(1).topRows(3);
	M.col(2).topRows(3) = a.cross(b);

#if 1
#if 1 // todo: for some reason the SVD version sometimes messes up the result...
	// pull out just the rotation component so we can normalize it.
	Matrix3d R = M.block<3,3>(0,0);
	JacobiSVD<Matrix3d> svd(R, ComputeFullU | ComputeFullV);

	// polar decomposition, R = (UV')(VSV')
	M.block<3,3>(0,0) = svd.matrixU()*svd.matrixV().transpose();
#else
	// kaess: going through quaternion instead of SVD - same problem...
	Eigen::Quaterniond quat = Rot3d::wRo_to_quat(M.block<3,3>(0,0));
	M.block<3,3>(0,0) = Rot3d::quat_to_wRo(quat);
#endif
#endif

	// Scale the results based on the scale in the homography. The
	// homography assumes that tags span from -1 to +1, i.e., that
	// they are two units wide (and tall).
	for (int i=0; i<3; ++i)
		M(i,3) *= tag_size/2;

	return M;
}

cv::Scalar rc() { // get a random pastel color
  Scalar s = cv::Scalar(150 + (float)rand()/((float)RAND_MAX/(100)),
                    	150 + (float)rand()/((float)RAND_MAX/(100)),
                    	150 + (float)rand()/((float)RAND_MAX/(100)));
  Scalar s1 = Scalar(s[0]/255, s[1]/255, s[2]/255);
  return s1;
}

cv::Scalar rc_dark() { // get a random pastel color
  Scalar s = cv::Scalar(75 + (float)rand()/((float)RAND_MAX/(75)),
                    	75 + (float)rand()/((float)RAND_MAX/(75)),
                    75 + (float)rand()/((float)RAND_MAX/(75)));
  Scalar s1 = Scalar(s[0]/255, s[1]/255, s[2]/255);
  return s1;
}

cv::Scalar rc_bright() { // get a random pastel color
	Scalar s = cv::Scalar(200 + (float)rand()/((float)RAND_MAX/(50)),
			200 + (float)rand()/((float)RAND_MAX/(50)),
			200 + (float)rand()/((float)RAND_MAX/(50)));
	Scalar s1 = Scalar(s[0]/255, s[1]/255, s[2]/255);
	return s1;
}

void get_color_table(map<int, Scalar> &color_table){

//	for(int i = 0; i < 36; i++){
//		color_table[i] = rc_bright();
//	}

	color_table[0] = Scalar(1, 0, 0);		// red
	color_table[1] = Scalar(0, 1, 0);		// green
	color_table[2] = Scalar(0, 0, 1);		// blue
	color_table[3] = Scalar(1, 1, 0);
	color_table[4] = Scalar(1, 0, 1);
	color_table[5] = Scalar(0, 1, 1);
	color_table[6] = Scalar(0.5, 0, 0);
	color_table[7] = Scalar(0, 0.5, 0);
	color_table[8] = Scalar(0, 0, 0.5);
	color_table[9] = Scalar(0.5, 0.5, 0);
	color_table[10] = Scalar(0, 0.5, 0.5);
	color_table[11] = Scalar(0.5, 0, 0.5);
	color_table[12] = Scalar(0.75, 0, 0);
	color_table[13] = Scalar(0, 0.75, 0);
	color_table[14] = Scalar(0, 0, 0.75);
	color_table[15] = Scalar(0.75, 0.75, 0);
	color_table[16] = Scalar(0.75, 0, 0.75);
	color_table[17] = Scalar(0, 0.75, 0.75);
	color_table[18] = Scalar(0.25, 0, 0);
	color_table[19] = Scalar(0, 0.25, 0);
	color_table[20] = Scalar(0, 0, 0.25);
	color_table[21] = Scalar(0.25, 0.25, 0);
	color_table[22] = Scalar(0, 0.25, 0.25);
	color_table[23] = Scalar(0.25, 0, 0.25);

	color_table[24] = Scalar(0.5, 0, 0);
	color_table[25] = Scalar(0, 0.5, 0);
	color_table[26] = Scalar(0, 0, 0.5);
	color_table[27] = Scalar(0.5, 0.5, 0);
	color_table[28] = Scalar(0, 0.5, 0.5);
	color_table[29] = Scalar(0.5, 0, 0.5);

}

/*!
 * Draw a annotation on im
 */
void draw_annotation(Mat &im, Rect bbox, string anno, Scalar color, int shift_x){


	rectangle( im, bbox.tl(), bbox.br(), color, 2, 8, 0 );

	// put text
	string text = anno;
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = 1;
	int thickness = 1;

	int baseline=0;
	Size textSize = getTextSize(text, fontFace,
	                            fontScale, thickness, &baseline);
	baseline += thickness;

	// center the text
	Point textOrg(bbox.x + shift_x, bbox.y - 8);

			// draw the box
	rectangle(im, textOrg + Point(0, baseline),
	          textOrg + Point(textSize.width, -textSize.height),
	          color, CV_FILLED);

	// then put the text itself
	putText(im, text, textOrg, fontFace, fontScale,
	        Scalar::all(0), thickness, 8);
}


}
