// file: listener.c
//
// LCM example program.

#include <stdio.h>
#include <inttypes.h>
#include <fstream>
#include <iostream>
#include <signal.h>

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <GL/gl.h>

#include <lcmtypes/bot_core_pose_t.h>
#include <lcmtypes/bot_core_image_t.h>
#include <lcmtypes/kinect_frame_msg_t.h>
#include <lcmtypes/obstacle_haptic_array_t.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge_lcm/rv-cv-bridge-lcm.h>

#include <kinect/kinect-utils.h>
#include <kinect/rv-kinect-frame-pcl-utils.h>

#include <jpeg-utils/jpeg-utils.h>
#include <jpeg-utils/jpeg-utils-ijg.h>

#include "rv-cv-pcl-utils.h"

lcm_t * lcm_;
cvBridgeLCM* cv_bridge;
KinectCalibration* kcal;
int kinect_decimate;
bot_lcmgl_t* lcmgl_;
bot_lcmgl_t* lcmgl_traj;
BotTrans *transform;
BotTrans *transform_maze;
std::map<int, cv::Scalar> color_table;

//////////////////////////////////////////////////////
// data analysis
int traj_id;

std::vector<Eigen::Vector3f> vec_pos;
std::vector<double> vec_yaw;
std::vector<double> vec_time;

// add data point every 10 messages
int win = 10;
int counter = 0;

float speed_max = 2;
float reori_max = 3.14;

// others
std::ofstream write_file;
std::vector<std::vector<float> > maze_segments;

void draw_traj(int traj_id){
	cv::Scalar s = color_table[traj_id];

	bot_lcmgl_push_matrix(lcmgl_traj);
	bot_lcmgl_translated(lcmgl_traj, 0, 0, 0);
	bot_lcmgl_line_width(lcmgl_traj, 10.0f);
	bot_lcmgl_point_size(lcmgl_traj, 2.0f);
	bot_lcmgl_enable(lcmgl_traj, GL_BLEND); // GL_BLEND, defined in usr/include/GL/gl.h


	for(int i = 0; i < vec_pos.size(); i++){
//		rv::draw_point_lcmgl(lcmgl_traj,
//				Eigen::Vector3d(vec_pos[i][0], vec_pos[i][1], vec_pos[i][2]),
//				s[0], s[1], s[2]);
		if(i > 0){
			rv::draw_line_lcmgl(lcmgl_traj,
					Eigen::Vector3d(vec_pos[i-1][0], vec_pos[i-1][1], vec_pos[i-1][2]),
					Eigen::Vector3d(vec_pos[i][0], vec_pos[i][1], vec_pos[i][2]),
					s[0], s[1], s[2]);
		}
	}

	bot_lcmgl_pop_matrix(lcmgl_traj);
	bot_lcmgl_switch_buffer(lcmgl_traj);
}


static void
vicon_handler(const lcm_recv_buf_t *rbuf, const char * channel,
        const bot_core_pose_t * msg, void * user)
{

    if(msg->pos[0] == 0 && msg->pos[1] == 0 && msg->pos[2] == 0){
    	return;
    }

    bot_core_pose_t p;
    p.utime = msg->utime;
    p.pos[0] = msg->pos[0];
    p.pos[1] = msg->pos[1];
    p.pos[2] = 0; // msg->pos[2]; // ignore height

    Eigen::Vector3f vis_p;
    vis_p << p.pos[0], p.pos[1], p.pos[2];

    double rpy_t[3];
	rpy_t[0] = 0; //msg->vel[0]; // ignore pitch and roll
	rpy_t[1] = 0; //msg->vel[1];
	rpy_t[2] = msg->vel[2];

	// to publish
    double quat_t[4];
    bot_roll_pitch_yaw_to_quat(rpy_t, quat_t);

    p.orientation[0] = quat_t[0];
    p.orientation[1] = quat_t[1];
    p.orientation[2] = quat_t[2];
    p.orientation[3] = quat_t[3];

    bot_core_pose_t_publish(lcm_, "POSE_BODY", &p);

    // to visualize
    transform->trans_vec[0] = p.pos[0];
    transform->trans_vec[1] = p.pos[1];
    transform->trans_vec[2] = p.pos[2];
    transform->rot_quat[0] = p.orientation[0];
    transform->rot_quat[1] = p.orientation[1];
    transform->rot_quat[2] = p.orientation[2];
    transform->rot_quat[3] = p.orientation[3];

    vec_pos.push_back(vis_p);
    vec_yaw.push_back(rpy_t[2]);
    vec_time.push_back(p.utime);

    draw_traj(traj_id);

	/////////////////////////////////////////////////////
	// data analysis
//	if(vec_pos.size() == 0){
//	    vec_pos.push_back(vis_p);
//	    vec_yaw.push_back(rpy_t[2]);
//	    vec_time.push_back(p.utime);
//	}
//
//	counter ++;
//
//	if(counter > win){
//
//		// reset counter
//		counter = 0;
//		draw_traj(traj_id);
//
//		// check that the movement is smaller than xxx
//		Eigen::Vector3f d_pos = vec_pos[vec_pos.size() - 1] - vis_p;
//
//		// usually < 0.02 for win = 10 for 130 Hz
//		//std::cout << d_pos.norm() << std::endl;
//		if(d_pos.norm() > speed_max){
//			std::cout << "unusual movement: " << d_pos.norm() << std::endl;
//			return;
//		}
//	    vec_pos.push_back(vis_p);
//	    vec_yaw.push_back(rpy_t[2]);
//	    vec_time.push_back(p.utime);
//
//
//	}

}

static void
kinect_frame_handler(const lcm_recv_buf_t *rbuf, const char * channel,
        const kinect_frame_msg_t * msg, void * user)
{
	cv::Mat im_rgb = cv::Mat::zeros(480, 640, CV_8UC3);

	// LCM message -> OpenCV Mat
	int rgb_buf_size_ = 640 * 480;
	uint8_t* rgb_data = (uint8_t*) malloc(rgb_buf_size_* 3);
	if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB) {
		memcpy(rgb_data, msg->image.image_data,
				msg->depth.width * msg->depth.height * 3);
	} else if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG) {
		jpeg_decompress_8u_rgb (msg->image.image_data, msg->image.image_data_nbytes,
				rgb_data, msg->image.width, msg->image.height, msg->image.width* 3);
		jpegijg_decompress_8u_rgb(msg->image.image_data, msg->image.image_data_nbytes,
		        rgb_data, msg->image.width, msg->image.height, msg->image.width* 3);
	}
	im_rgb.data = rgb_data;

	cv_bridge->publish_mjpg(im_rgb, (char*)"IMAGE_VIS");

	// MSG -> cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	rv::unpack_kinect_frame(msg, rgb_data, kcal, kinect_decimate, cloud_raw);

	free(rgb_data);
}

static void
obstacle_haptic_array_handler(const lcm_recv_buf_t *rbuf, const char * channel,
        const obstacle_haptic_array_t * msg, void * user)
{
//	vector<double> ranges_m;
//	for(int i = 0; i < msg->num_ranges; i++){
//		ranges_m[i] = msg->ranges_m[i];
//	}
	//std::cout << "receive haptic mesage" << std::endl;

	//////////////////////////////////////////////////////////////////
	// visualizae PointCloud
    bot_core_pose_t p;
    p.utime = msg->utime;
    p.pos[0] = 0.2; //msg->trans_xyzrpy[0];
    p.pos[1] = 0; //msg->trans_xyzrpy[1];
    p.pos[2] = -msg->ground_height; //msg->trans_xyzrpy[2];

    // those may be rpy now
    double rpy_t[3];
	rpy_t[0] = -90 / 180.0 * M_PI; //msg->trans_xyzrpy[3];// -90 / 180.0 * M_PI; //msg->vel[0]; // ignore pitch and roll
	rpy_t[1] = 0; //msg->trans_xyzrpy[4]; //0 / 180.0 * M_PI; //msg->vel[1];
	rpy_t[2] = -90 / 180.0 * M_PI; //msg->trans_xyzrpy[5];// -90 / 180.0 * M_PI;
    double quat_t[4];
    bot_roll_pitch_yaw_to_quat(rpy_t, quat_t);

	double rpy_h[3];
	rpy_h[0] = msg->trans_xyzrpy[3];// //msg->vel[0]; // ignore pitch and roll
	rpy_h[1] = msg->trans_xyzrpy[4]; //30 / 180.0 * M_PI; //msg->vel[1];
	rpy_h[2] = msg->trans_xyzrpy[5];// -90 / 180.0 * M_PI;
    double quat_h[4];
    bot_roll_pitch_yaw_to_quat(rpy_h, quat_h);

    double quat_out[4];
    bot_quat_mult(quat_out, quat_h, quat_t);
    p.orientation[0] = quat_out[0];
    p.orientation[1] = quat_out[1];
    p.orientation[2] = quat_out[2];
    p.orientation[3] = quat_out[3];

    bot_core_pose_t_publish(lcm_, "POSE_HAPTIC", &p);


    //////////////////////////////////////////////////////////////
    // "control" signals

    // 5 haptic motors
    bot_lcmgl_translated(lcmgl_, 0, 0, 0);
    bot_lcmgl_line_width(lcmgl_, 2.0f);
    bot_lcmgl_point_size(lcmgl_, 12.0f);
    bot_lcmgl_begin(lcmgl_, GL_POINTS);

    Eigen::Vector3d m0 = Eigen::Vector3d(-1, .5, 1.0);
    for(int i = 0; i < 5; i++){
    	Eigen::Vector3d m = m0 + Eigen::Vector3d(0, -.25 * i, 0);
        double m2[3] = {m[0], m[1], m[2]};
        double m3[3];
        bot_trans_apply_vec(transform, m2, m3);

        if(msg->intensities[i] == 7){
        	bot_lcmgl_color3f(lcmgl_, 1.0, 0, 0);
        }else{
        	bot_lcmgl_color3f(lcmgl_, 0, 0, 1.0);
        }


        bot_lcmgl_vertex3f(lcmgl_, m3[0], m3[1], m3[2]);
    }


    bot_lcmgl_end(lcmgl_);
    bot_lcmgl_switch_buffer(lcmgl_);

}


void get_maze_segments(std::string csv_file, std::vector<std::vector<float> > & maze_segments){

	std::vector<float> segments, segments_t;
	std::ifstream file (csv_file);
	std::string value;

	std::vector<Eigen::Vector3d> holodeck_corners;
	Eigen::Vector3d ll, lr, tr, tl;
	ll << -2.575, -1.945, 0;
	lr << 2.565, -2,022, 0;
	tr << 2.480, 3.180, 0;
	tl << -2.588, 3.121, 0;

	double dx, dy, sx, sy;
	dx = 0.5 * (lr[0] - ll[0]) + 0.5 * (tr[0] - tl[0]);
	dy = 0.5 * (tr[1] - lr[1]) + 0.5 * (tl[1] - ll[1]);

	sx = dx / 4;
	sy = dy / 4;

	while ( file.good() )
	{

		float y1, x1, y2, x2;
		getline ( file, value, ',' );
		x1 = atof(value.c_str()) * sx + ll[1];
		getline ( file, value, ',' );
		y1 = (4 - atof(value.c_str())) * sy + ll[0];
		getline ( file, value, ',' );
		x2 = atof(value.c_str()) * sx + ll[1];
		getline ( file, value);
		y2 = (4 - atof(value.c_str())) * sy + ll[0];

		segments.push_back(y1);
		segments.push_back(x1);
		segments.push_back(y2);
		segments.push_back(x2);

		maze_segments.push_back(segments);

		segments.clear();
	}

}

void termination_handler(int signum)
{
	// data-analysis
	double total_traj_length 		= 0;
	double total_traj_rot 			= 0;
	double total_traj_time 			= 0;

	for(int i = 0; i < vec_pos.size(); i++){
		if(i > 0){
			Eigen::Vector3d pt_st, pt_end;
			pt_st 	<< vec_pos[i-1][0], vec_pos[i-1][1], vec_pos[i-1][2];
			pt_end 	<< vec_pos[i][0], vec_pos[i][1], vec_pos[i][2];

			Eigen::Vector3d d_pts = pt_end - pt_st;
			total_traj_length += d_pts.norm();

			double d_yaw = (double)fabs(vec_yaw[i] - vec_yaw[i-1]);
			if(d_yaw < reori_max){
				total_traj_rot += d_yaw;
			}else if(d_yaw > 5){
				std::cout << "swithch +/-" <<  std::endl;
			}else{
				std::cout << "out of reorientation limit." << std::endl;
				std::cout << "traj rot\t" 	<<  vec_yaw[i] << "\t" << vec_yaw[i-1] <<  std::endl;
			}

			total_traj_time += (vec_time[i] - vec_time[i-1]);
			//std::cout << "traj time\t" 	<<  total_traj_time << std::endl;
		}
	}

	std::cout << std::endl;
	std::cout << "traj length\t" << total_traj_length << std::endl;
	std::cout << "traj rot\t" 	<<  total_traj_rot << std::endl;
	// this is the time from republished LCM, not original utime
	std::cout << "traj time\t" 	<<  total_traj_time / 1000000 << std::endl;

	std::cout << "program ended" << std::endl;
	write_file.close();


	exit(0);
}

void setup_signal_handlers(void (*handler)(int))
{
	struct sigaction new_action, old_action;
	memset(&new_action, 0, sizeof(new_action));
	new_action.sa_handler = handler;
	sigemptyset(&new_action.sa_mask);

	// Set termination handlers and preserve ignore flag.
	sigaction(SIGINT, NULL, &old_action);
	if (old_action.sa_handler != SIG_IGN)
		sigaction(SIGINT, &new_action, NULL);
	sigaction(SIGHUP, NULL, &old_action);
	if (old_action.sa_handler != SIG_IGN)
		sigaction(SIGHUP, &new_action, NULL);
	sigaction(SIGTERM, NULL, &old_action);
	if (old_action.sa_handler != SIG_IGN)
		sigaction(SIGTERM, &new_action, NULL);
}

static void
usage(const char *name)
{
	fprintf(
			stderr,
			"usage: %s [options]\n"
			"\n"
			"  -h, --help                            Shows this help text and exits\n"
			"  -j, --traj-id	                     traj id (1-5)\n"
			"  -w, --win-size	                     intevals\n"
			"  -s, --speed-max	                     speed max \n"
			"  -r, --reorientation-max	             reorientation max \n"
			"\n",
			name);

	exit(1);
}

int
main(int argc, char ** argv)
{

    lcm_ = lcm_create(NULL);
    cv_bridge = new cvBridgeLCM(lcm_, lcm_);
	kcal = kinect_calib_new();
	rv::get_default_kinect_calib(kcal);
	kinect_decimate =2.0;

	rv::get_color_table(color_table);

	traj_id = 1;
	win = 10;

	const char *optstring = "hj:w:s:r:";
	int c;
	struct option long_opts[] =
	{
			{ "help", no_argument, 0, 'h' },
			{ "traj", required_argument, 0, 'j' },
			{ "win", required_argument, 0, 'w' },
			{ "speed", required_argument, 0, 's' },
			{ "reorientation", required_argument, 0, 'r' },
			{ 0, 0, 0, 0 }
	};

	while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
		switch (c){
		case 'j':
			traj_id = atoi(optarg);
			break;
		case 'w':
			win = atoi(optarg);
			break;
		case 's':
			speed_max = atof(optarg);
			break;
		case 'r':
			reori_max = atof(optarg);
			break;
		case 'h':
			usage(argv[0]);
			break;
		}
	}

    if(!lcm_)
        return 1;

    bot_core_pose_t_subscription_t * sub =
        bot_core_pose_t_subscribe(lcm_, "POSE", &vicon_handler, NULL);

    kinect_frame_msg_t_subscription_t * sub1 =
        kinect_frame_msg_t_subscribe(lcm_, "KINECT_FRAME", &kinect_frame_handler, NULL);

    obstacle_haptic_array_t_subscription_t * sub2 =
    		obstacle_haptic_array_t_subscribe(lcm_, "HAPTIC_ARRAY", &obstacle_haptic_array_handler, NULL);

    lcmgl_ = bot_lcmgl_init(lcm_, "LCMGL_VIS");

    std::stringstream ss_traj;
    ss_traj << "LCMGL_TRAJ_" << traj_id;
    lcmgl_traj = bot_lcmgl_init(lcm_, ss_traj.str().c_str());

    transform = new BotTrans();
    transform_maze = new BotTrans();

	setup_signal_handlers(termination_handler);

    while(1)
        lcm_handle(lcm_);

    bot_core_pose_t_unsubscribe(lcm_, sub);
    kinect_frame_msg_t_unsubscribe(lcm_, sub1);
    obstacle_haptic_array_t_unsubscribe(lcm_, sub2);

    lcm_destroy(lcm_);
    return 0;
}

