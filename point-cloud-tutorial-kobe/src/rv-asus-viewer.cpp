#include "rv-kinect-frame-pcl-lcm-simple.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <sys/time.h>
using namespace cv;
using namespace std;

bool finish_;
double duration;                    //Variable time duration
struct timeval start;               //Variable start time
struct timeval stop;                //Variable stop time
blKinectFramePCL::blKinectFramePCL(lcm_t* lcm) {

	this->lcm_ = lcm;

	kcal = kinect_calib_new();
	rv::get_default_kinect_calib(kcal);
//	rv::get_default_ti_board_calib(kcal);
	kinect_decimate =4.0;

	cv_bridge = new cvBridgeLCM(lcm, lcm);
	
	string frame_channel_ = "KINECT_FRAME";
	kinect_frame_msg_t_subscription_t* sub_kinect = kinect_frame_msg_t_subscribe(
			lcm_, frame_channel_.c_str(), blKinectFramePCL::on_kinect_frame_aux, this);
	cout << "Subscribe kinect_frame_msg_t channel:\t" << frame_channel_ << endl;

	this->lcmgl_ = bot_lcmgl_init(lcm_, "LCMGL");
	this->lcmgl_spotlight = bot_lcmgl_init(lcm_, "LCMGL_SPOTLIGHT");
	this->lcmgl_haptic = bot_lcmgl_init(lcm_, "LCMGL_HAPTIC");
	this->lcmgl_v = bot_lcmgl_init(lcm_, "LCMGL_VERTICAL");
	this->lcmgl_h = bot_lcmgl_init(lcm_, "LCMGL_HORIZONTAL");
	this->lcmgl_pointcloud = bot_lcmgl_init(lcm_, "LCMGL_DEMO");
	this->is_init = 0;
	this->timestamp_init = bot_timestamp_now();
	this->default_pitch = 20.0;
	this->default_height = 1.0;

	this->publish_lcmgl = 1;

	this->start_time = bot_timestamp_now();
	this->adjust_ground_height_time = 5;
	
}

void blKinectFramePCL::set_pitch(float pitch){
	this->default_pitch = pitch;
}

void blKinectFramePCL::set_height(float height){
	this->default_height = height;
}

void blKinectFramePCL::set_fov(int h_fov, int v_fov){
	this->h_fov = h_fov;
	this->v_fov = v_fov;
}

void blKinectFramePCL::set_decimate(int decimate){
	this->kinect_decimate = decimate;
}

void blKinectFramePCL::set_haptic_settings(
		float h_min_r, float h_max_r, char* h_jn, int vibration_pattern){
	this->haptic_min_range = h_min_r;
	this->haptic_max_range = h_max_r;
	stringstream ss;
	ss << h_jn;
	string h_jn_str = ss.str();
	if(h_jn_str.size() != this->motor_num){
		cout << "haptic just notice setting should have the same digit number as motor number" << endl;
		exit(0);
	}
	for(int i = 0; i < h_jn_str.size(); i++){
		int jn = atoi( h_jn_str.substr(i, 1).c_str() );
		if(jn > 7 || jn <0){
			cout << "haptic just notice setting is should be 0-7" << endl;
			exit(0);
		}
		this->haptic_just_notice_array.push_back(jn);
	}
	this->haptic_vibration_pattern = vibration_pattern;
}

void blKinectFramePCL::set_braille_display_mode(int mode){
	this->braille_display_mode = mode;
}

void blKinectFramePCL::set_haptic_array_mode(int mode){
	this->haptic_array_mode = mode;
}

void blKinectFramePCL::set_motor_num(int num){
	this->motor_num = num;
}

void blKinectFramePCL::set_publish_lcmgl(int i){
	this->publish_lcmgl = i;
}

void blKinectFramePCL::set_patch_density(float patch_density){
	this->patch_density = patch_density;
}

void blKinectFramePCL::set_rotated(int is_rotated){
	this->is_rotated = is_rotated;
}

void blKinectFramePCL::set_adjust_ground_height_time(int adjust_ground_height_time){
	this->adjust_ground_height_time = adjust_ground_height_time;
}



blKinectFramePCL::~blKinectFramePCL() {
}

void blKinectFramePCL::on_frame(const kinect_frame_msg_t* msg) {

    double t_tot = (double)cvGetTickCount();

	/////////////////////////////////////////////////////////////
	// State Variable
	////////////////////////////////////////////////////////////
   
	// decripted
	// 1: no input
	// 2: cannot see ground
	// 3: ??
	// 4: uneven ground
	int state_id = 0;
    
	// individual directions
    int num_ranges = this->motor_num;

    string tts_data = "";

    
    gettimeofday(&start,NULL);
    duration=(start.tv_sec-stop.tv_sec)+(start.tv_usec-stop.tv_usec)/1000000.0;
	printf("Waiting Time:\t%lf\t",duration);
	///////////////////////////////////////
	// 1. Input Pre-processing
	// Input: msg
	// Outputs: cloud_raw, im_rgb, im_depth_vis
	//////////////////////////////////////

	int rgb_buf_size_ = 640 * 480;
	cv::Mat im_rgb = cv::Mat(480, 640, CV_8UC3);
	Mat im_depth_vis = cv::Mat::zeros(480, 640, CV_8UC3);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw_reg = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PCLPointCloud2 blob;
	uint8_t* rgb_buf_ = (uint8_t*) malloc(rgb_buf_size_* 3);

	rv::unpack_kinect_frame(msg, rgb_buf_, this->kcal, this->kinect_decimate, cloud_raw);
	pcl::copyPointCloud (*cloud_raw,*cloud_raw_reg);
	im_rgb.data = rgb_buf_;
	
	////////////////////////////////save to pcd file///////////////////////////
	if(msg->depth.depth_data_format == KINECT_DEPTH_MSG_T_DEPTH_MM){
		//Bot-lcm-viewer
		bot_lcmgl_translated(lcmgl_pointcloud, 0, 0, 0);
    		bot_lcmgl_line_width(lcmgl_pointcloud, 4.0f);
    		bot_lcmgl_point_size(lcmgl_pointcloud, 6.0f);
    		bot_lcmgl_begin(lcmgl_pointcloud, GL_POINTS);




		for(int i=0;i<cloud_raw->points.size ();i++){
				//printf("x= %f y=%f z=%f\n ",cloud_cluster_don->points[i].x,cloud_cluster_don->points[i].y,cloud_cluster_don->points[i].z);
				bot_lcmgl_color3f(lcmgl_pointcloud, cloud_raw->points[i].r/255.0, cloud_raw->points[i].g/255.0, cloud_raw->points[i].b/255.0);
    			bot_lcmgl_vertex3f(lcmgl_pointcloud, cloud_raw->points[i].x, cloud_raw->points[i].y, cloud_raw->points[i].z);	    
    			printf("%f\t%f\t%f\n",cloud_raw->points[i].x,cloud_raw->points[i].y,cloud_raw->points[i].z);
		   }

		
			//bot_lcmgl_end(lcmgl_pointcloud);
    		bot_lcmgl_switch_buffer(lcmgl_pointcloud);
    		
    }
    	gettimeofday(&stop,NULL);
		duration=(stop.tv_sec-start.tv_sec)+(stop.tv_usec-start.tv_usec)/1000000.0;
		printf("CPU Time:\t%lf\n",duration);
    		

		/////////////////////////////////////Cloud segmentation/////////////////////////////////////////////////////

		

}

void blKinectFramePCL::on_kinect_frame_aux(const lcm_recv_buf_t* rbuf,
		const char* channel,
		const kinect_frame_msg_t* msg,
		void* user_data) {
	(static_cast<blKinectFramePCL *>(user_data))->on_frame(msg);
}


//////////////////////////////////////
// MAIN
//////////////////////////////////////

blKinectFramePCL* kf_pcl;

void termination_handler(int signum)
{
	finish_ = true;
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
			"  -s, --show-spotlight                  show spotlight, default: 1\n"
			"  -p, --pitch                           add pitch (degree), default: 10\n"
			"  -z, --z-height                        add height (meter), default: 1.0\n"
			"  -g, --adjust-ground-height            adjust ground height, 0: always, > 0: N seconds\n"
			"  -f, --h-fov                           horizontal fov (degree), default: 60\n"
			"  -v, --v-fov                           vertical fov (meter), default: 40\n"
			"  -d, --decimate                        decimate, default: 2\n"
			"  -r, --rotated                         rotated\n"
			"  -a, --haptic-array                    Haptic Array Mode. 1: range, 2: height\n"
			"  -j, --haptic-just-notice              Haptic just notice\n"
			"  -m, --haptic-min-range                Haptic min range\n"
			"  -x, --haptic-max-range                Haptic max range\n"
			"  -b, --haptic-vibration-pattern        Haptic vibration pattern\n"
			"  -c, --braille-display-mode            Braille display mode\n"
			"  -n, --motor-num                       Number of Vibration Motor. default: 5\n"
			"  -e, --patch-density                   haptic point check. default: 0.3\n"
			"  -l, --lcmgl                           publish lcmgl. default: 0\n"
			"\n",
			name);

	exit(1);
}

int main(int argc, char** argv)
{
	finish_ = false;
	int publish_lcmgl = 0;

	float pitch = 30;
	float height = 1.0;
	int h_fov = 60;
	int v_fov = 40;
	int decimate = 4.0;
	int is_rotated = 0;

	int adjust_ground_height_time = 5;

	int haptic_array_mode = 2;
	int haptic_motor_num = 5;
	float patch_density = 0.3;

	char* haptic_just_notice = (char*)"33333";
	float haptic_min_range = 1.0;
	float haptic_max_range = 2.4;
	int haptic_vibration_pattern = 1;

	int braille_display_mode = 2;
	
	const char *optstring = "hs:p:z:f:v:d:a:n:l:e:r:j:m:x:b:c:g:";
	int c;
	struct option long_opts[] =
	{
			{ "help", no_argument, 0, 'h' },
			{ "show-spotlight", required_argument, 0, 's' },
			{ "pitch", required_argument, 0, 'p' },
			{ "z-height", required_argument, 0, 'z' },
			{ "adjust-ground-height", required_argument, 0, 'g' },
			{ "h-fov", required_argument, 0, 'f' },
			{ "v-fov", required_argument, 0, 'v' },
			{ "rotated", required_argument, 0, 'r' },
			{ "decimate", required_argument, 0, 'd' },
			{ "haptic-array", required_argument, 0, 'a' },
			{ "haptic-num", required_argument, 0, 'n' },
			{ "publish-lcmgl", required_argument, 0, 'l' },
			{ "patch-density", required_argument, 0, 'e' },
			{ "haptic-just-notice", required_argument, 0, 'j' },
			{ "haptic-min-range", required_argument, 0, 'm' },
			{ "haptic-max-range", required_argument, 0, 'x' },
			{ "haptic-vibration-pattern", required_argument, 0, 'b' },
			{ "braille-display-mode", required_argument, 0, 'c' },
			{ 0, 0, 0, 0 }
	};

	while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
		switch (c){
		case 'p':
			pitch = atof(optarg);
			break;
		case 'z':
			height = atof(optarg);
			break;
		case 'f':
			h_fov = atoi(optarg);
			break;
		case 'g':
			adjust_ground_height_time = atoi(optarg);
			break;
		case 'v':
			v_fov = atoi(optarg);
			break;
		case 'd':
			decimate = atoi(optarg);
			break;
		case 'r':
			is_rotated = atoi(optarg);
			break;
		case 'a':
			haptic_array_mode = atoi(optarg);
			break;
		case 'n':
			haptic_motor_num = atoi(optarg);
			break;
		case 'l':
			publish_lcmgl = atoi(optarg);
			break;
		case 'e':
			patch_density = atof(optarg);
			break;
		case 'j':
			haptic_just_notice = optarg;
			break;
		case 'm':
			haptic_min_range = atof(optarg);
			break;
		case 'x':
			haptic_max_range = atof(optarg);
			break;
		case 'b':
			haptic_vibration_pattern = atoi(optarg);
			break;
		case 'c':
			braille_display_mode = atoi(optarg);
			break;
		case 'h':
			usage(argv[0]);
			break;
		}
	}

	// "udpm://239.255.76.67:7667?ttl=1"
	lcm_t* lcm = lcm_create(NULL);

	kf_pcl = new blKinectFramePCL(lcm);
	kf_pcl->set_pitch(pitch);
	kf_pcl->set_height(height);
	kf_pcl->set_fov(h_fov, v_fov);
	kf_pcl->set_decimate(decimate);
	kf_pcl->set_haptic_array_mode(haptic_array_mode);
	kf_pcl->set_motor_num(haptic_motor_num);
	kf_pcl->set_publish_lcmgl(publish_lcmgl);
	kf_pcl->set_patch_density(patch_density);
	kf_pcl->set_rotated(is_rotated);
	kf_pcl->set_haptic_settings(haptic_min_range, haptic_max_range,
			haptic_just_notice, haptic_vibration_pattern);
	kf_pcl->set_adjust_ground_height_time(adjust_ground_height_time);
	kf_pcl->set_braille_display_mode(braille_display_mode);

	setup_signal_handlers(termination_handler);
	while(0 == lcm_handle(lcm) && finish_ == false) ;

	delete kf_pcl;

    printf("Dosvedanya!\n");
    return 0;
}


