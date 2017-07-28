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
struct timeval start1;               //Variable start time
struct timeval stop1;                //Variable stop time
blKinectFramePCL::blKinectFramePCL(lcm_t* lcm) {

	this->lcm_ = lcm;

	kcal = kinect_calib_new();
	rv::get_default_kinect_calib(kcal);
//	rv::get_default_ti_board_calib(kcal);
	kinect_decimate =10.0;

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
void pointcloudResize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	int count=0;
	for(int i=0;i<cloud->points.size();i++){
		   	if(cloud->points[i].x<100 ){
		   		cloud->points[count].x = cloud->points[i].x;
		   		cloud->points[count].y = cloud->points[i].y;
		   		cloud->points[count].z = cloud->points[i].z;

		   		count++;
		   	}
	}
	 cloud->points.resize(count);
	 return;
}
void pointcloudResize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	int count=0;
	for(int i=0;i<cloud->points.size();i++){
		   	if(cloud->points[i].x<100 ){
		   		cloud->points[count].x = cloud->points[i].x;
		   		cloud->points[count].y = cloud->points[i].y;
		   		cloud->points[count].z = cloud->points[i].z;
		   		cloud->points[count].r = cloud->points[i].r;
		   		cloud->points[count].g = cloud->points[i].g;
		   		cloud->points[count].b = cloud->points[i].b;
		   		//cloud_raw->points[count].r = doncloud->points[i].curvature*255;

		   		count++;
		   	}
	}
	 cloud->points.resize(count);
	 return;
}
void pointcloudResize(pcl::PointCloud<pcl::PointNormal>::Ptr cloud){
	int count=0;
	for(int i=0;i<cloud->points.size();i++){
		   	if(cloud->points[i].x<100 ){
		   		cloud->points[count].x = cloud->points[i].x;
		   		cloud->points[count].y = cloud->points[i].y;
		   		cloud->points[count].z = cloud->points[i].z;
		   		cloud->points[count].curvature = cloud->points[i].curvature;
		   		//cloud_raw->points[count].r = doncloud->points[i].curvature*255;

		   		count++;
		   	}
	}
	 cloud->points.resize(count);
	 return;
}

///////////////Sort the pointcloud in line segment and filter out the outlier points
void line_segment_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::ModelCoefficients line){
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered= pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_reg= pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);;
	
	pcl::copyPointCloud (*cloud,*cloud_filtered);
	vector<double> cloud_x(cloud->points.size(), 0);   
	vector<int> index(cloud->points.size(), 0);

	for (int i = 0 ; i <cloud->points.size() ; i++) {
		if(fabs(line.values[3]/line.values[4]) >=1 )
			cloud_x[i]=(cloud->points[i].x);
		else
			cloud_x[i]=(cloud->points[i].y);
	    index[i] = i;
	}
	sort(index.begin(), index.end(),
	    [&](const int& a, const int& b) {
	        return (cloud_x[a] < cloud_x[b] );
	    }
	);
	for (int i = 0 ; i < index.size() ; i++) {
		if(fabs(line.values[3]/line.values[4] )>=1 ){
	    	cloud_filtered->points[i].x=cloud_x[index[i]];
	    	cloud_filtered->points[i].y=cloud->points[index[i]].y;
		}
		else{
	    	cloud_filtered->points[i].y=cloud_x[index[i]];
	    	cloud_filtered->points[i].x=cloud->points[index[i]].x;
		}
	    	
	    cloud_filtered->points[i].z=cloud->points[index[i]].z;
	}
	pcl::copyPointCloud (*cloud_filtered,*cloud_reg);
	for (int i = 1 ; i < index.size() ; i++) {
	    double length=sqrt((cloud_filtered->points[i].x-cloud_filtered->points[i-1].x)*(cloud_filtered->points[i].x-cloud_filtered->points[i-1].x)+\
	    (cloud_filtered->points[i].y-cloud_filtered->points[i-1].y)*(cloud_filtered->points[i].y-cloud_filtered->points[i-1].y));
	    	if(length>2){
	    		//printf("i:%d\n",i);
	    		if(i>index.size()-i){
	    			for(int j=i;j!=index.size() ; j++)
	    				cloud_reg->points[j].x=200;
	    		}
	    		else{
	    			for(int j=0;j<i ; j++)
	    				cloud_reg->points[j].x=200;
	    		}
	    	}
	    
	}
	pointcloudResize(cloud_reg);
	pcl::copyPointCloud (*cloud_reg,*cloud);
	//cloud=cloud_reg;
	
	 return;
}



blKinectFramePCL::~blKinectFramePCL() {
}

void blKinectFramePCL::on_frame(const kinect_frame_msg_t* msg) {

    double t_tot = (double)cvGetTickCount();
    double duration1;
	/////////////////////////////////////////////////////////////
	// State Variable
	////////////////////////////////////////////////////////////
   
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
	double final_right_direction[2];		final_right_direction[0]=0; final_right_direction[1]=0;
	double final_right_distance=0;
	double final_left_direction[2];		final_left_direction[0]=0; final_left_direction[1]=0;
	double final_left_distance=0;
	////////////////////////////////save to pcd file///////////////////////////
	if(msg->depth.depth_data_format == KINECT_DEPTH_MSG_T_DEPTH_MM){
		gettimeofday(&start1,NULL);
		//Bot-lcm-viewer
		bot_lcmgl_translated(lcmgl_pointcloud, 0, 0, 0);
    	bot_lcmgl_line_width(lcmgl_pointcloud, 4.0f);
    	bot_lcmgl_point_size(lcmgl_pointcloud, 6.0f);
    	bot_lcmgl_begin(lcmgl_pointcloud, GL_POINTS);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
  		cloud_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointXYZ minPt, maxPt;
    	pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZ>(*cloud_raw, *cloud_xyz);
		pcl::getMinMax3D (*cloud_xyz, minPt, maxPt);
  		std::cout << "Max x: " << maxPt.x << "Max y: " << maxPt.y << "Max z: " << maxPt.z<<"Min x: " <<minPt.x << "Min y: " << minPt.y << "Min z: " << minPt.z << std::endl;
		/////////////////////////////////////Cloud segmentation/////////////////////////////////////////////////////
    		

    	/////////filter out the too far clouds
		  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
		  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond   (new  pcl::ConditionAnd<pcl::PointXYZRGB> ());
  		  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new   pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, minPt.z)));		//-1.3
		  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new   pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, minPt.z+1.2)));		//-0.3
		  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new   pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, 5.0)));		//-0.3
		  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new   pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::GT, -5.0)));		//-0.3
		  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GT, 0.0)));			//zed = 2.2
		  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, 8.0)));			//zed = 2.2
		  // build the filter
		  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem1 (range_cond);
		  condrem1.setInputCloud (cloud_raw);
		  condrem1.setKeepOrganized(true);
		  condrem1.filter (*cloud_filtered);
		  printf("points:%lf\n",cloud_filtered->points.size());
		  pointcloudResize(cloud_filtered);
		  printf("points:%lf\n",cloud_filtered->points.size());


		  int reg2=(int)((minPt.z)/0.05)-1;
		  int reg1=(int)((minPt.z+0.8)/0.05)+1;
		  double height[reg1-reg2+1];
		  double height_hit[reg1-reg2+1];
		  for(int i=0;i<reg1-reg2+1;i++){
		  	height[i]=minPt.z+0.05*i;
		  	height_hit[i]=0;
		  }
		  	
		  for(int i=0;i<cloud_filtered->points.size();i++){
		  	for(int j=0;j<reg1-reg2;j++){
		  		if(cloud_filtered->points[i].z<height[j+1] && cloud_filtered->points[i].z>=height[j])
		  			height_hit[j]++;
		  	}
		  }
		  for(int i=0;i<reg1-reg2;i++){
		  		height_hit[i]=height_hit[i]/cloud_filtered->points.size();
		  		printf("%lf < z < %lf = %lf\n",height[i],height[i+1],height_hit[i]);
		  }
		  double h_percent=0;
		  int h_index=0;
		  for(int i=0;i<reg1-reg2;i++){
		  		height_hit[i]=height_hit[i];
		  		printf("%lf < z < %lf = %lf\n",height[i],height[i+1],height_hit[i]);
		  		if(h_percent < 0.4){
		  			h_percent += height_hit[i];
		  			h_index++;
		  		}
		  		
		  }
		  printf("h_index:%d\n",h_index);
		  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new   pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, height[h_index+2])));		//-1.3
		  condrem1.setInputCloud (cloud_filtered);
		  condrem1.setKeepOrganized(true);
		  condrem1.filter (*cloud_filtered);
		  pointcloudResize(cloud_filtered);


		 // cloud_raw=cloud_filtered;
		  cloud_raw_reg=cloud_filtered;

		  gettimeofday(&stop1,NULL);
		  duration1=(stop1.tv_sec-start1.tv_sec)+(stop1.tv_usec-start1.tv_usec)/1000000.0;
		  printf("Time Far pointcloud filtering out:\t%lf\n",duration1);
		  gettimeofday(&start1,NULL);




		// Create a search tree, use KDTreee for non-organized data.
		  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;		  
		  if (cloud_raw_reg->isOrganized ())
		    tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB> ());
		  else
		    tree.reset (new pcl::search::KdTree<pcl::PointXYZRGB> (false));
			
		  // Set the input pointcloud for the search tree
		  tree->setInputCloud (cloud_raw_reg);	
		  // Compute normals using both small and large scales at each point
		  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne1;
		  ne1.setInputCloud (cloud_raw_reg);
		  ne1.setSearchMethod (tree);

		 
		  // * NOTE: setting viewpoint is very important, so that we can ensure
		  // * normals are all pointed in the same direction!	   
		  ne1.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

		  // calculate normals with the small scale
		  pcl::PointCloud <pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);
		  ne1.setRadiusSearch (0.2);			//zed = 0.05
		  ne1.compute (*normals_small_scale);
		  // calculate normals with the large scale
		  pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);
		  ne1.setRadiusSearch (0.4);			//zed = 0.14
		  ne1.compute (*normals_large_scale);
		

		  // Create output cloud for DoN results
		  pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
		  pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointNormal>(*cloud_raw_reg, *doncloud);
		  /*
		  for(int i=0;i<doncloud->points.size();i++){
		  
		  	printf("%lf , %lf,%lf,%lf \n",doncloud->points[i].x,doncloud->points[i].y,doncloud->points[i].z,doncloud->points[i].curvature);
		}*/

		  // Create DoN operator
		  pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::PointNormal> don;
		  don.setInputCloud (cloud_raw_reg);
		  don.setNormalScaleLarge (normals_large_scale);				//Use the large range point cloud to calculate the normal 
		  don.setNormalScaleSmall (normals_small_scale);				//Use the small range point cloud to calculate the normal 
		  // Compute DoN
		  don.computeFeature (*doncloud);								//Calculate Difference of normal
		  //printf("point number _ before resize: %d\n",doncloud->points.size());
		  pointcloudResize (cloud_raw_reg);
		 // pointcloudResize (doncloud);
		  //printf("Epoint number after resize: %d\n",doncloud->points.size());
		  pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_left=doncloud;
		  pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_right=doncloud;
		  /*
for(int i=0;i<doncloud->points.size();i++){
		  
		  	printf("%lf , %lf,%lf,%lf \n",doncloud->points[i].x,doncloud->points[i].y,doncloud->points[i].z,doncloud->points[i].curvature);
		}
		*/
		  //////////////////Calculate the histogram of "Difference of Normal"
		double curpercent[10];
		double curvatur_hit[10];
		for(int i=0;i<10;i++)
		  	curvatur_hit[i]=0;
		for(int i=0;i<doncloud->points.size();i++){
		  	double a=0;
		  	for(int j=0;j<10;j++){
		  		if( doncloud->points[i].curvature <0.1*j+0.1 && doncloud->points[i].curvature>=0.1*j)
		  			curvatur_hit[j]++;
		  		if( doncloud->points[i].curvature >1.0)
		  			printf("ERROR: %lf\n",doncloud->points[i].curvature);
		  	}		  	
		}
		  printf("Epoint number: %d\n",doncloud->points.size());
		for(int i=0;i<10;i++){
		  	curpercent[i]=curvatur_hit[i]/doncloud->points.size();
		  	printf("%lf < curvature < %lf = %lf\n",0.1*i,0.1*(i+1),curpercent[i]);
		}
		  
		  
		
		  ///////////////////////Filter out the high curvature(ground line) and ground.

		  // Build the condition for filtering			filter the right road
		  pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond_right (new pcl::ConditionAnd<pcl::PointNormal> () );
		  range_cond_right->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::LT, 0.15)));//0.2 
		  range_cond_right->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new pcl::FieldComparison<pcl::PointNormal> ("z", pcl::ComparisonOps::GT, height[h_index+2])));
		  // Build the filter
		  pcl::ConditionalRemoval<pcl::PointNormal> condrem_right (range_cond_right);
		  condrem_right.setInputCloud (doncloud_right);
		  condrem_right.setKeepOrganized(true);
		   pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered_right (new pcl::PointCloud<pcl::PointNormal>);
		  // Apply filter

		  condrem_right.filter (*doncloud_filtered_right);
		  doncloud_right = doncloud_filtered_right;
		  
		  pointcloudResize(doncloud_right);
		 
 		gettimeofday(&stop1,NULL);
		  duration1=(stop1.tv_sec-start1.tv_sec)+(stop1.tv_usec-start1.tv_usec)/1000000.0;
		  printf("Time filter out ground:\t%lf\n",duration1);
		  gettimeofday(&start1,NULL);

		 	
		
		   pcl::PointCloud<pcl::PointXYZRGB>::Ptr doncloud1_right (new pcl::PointCloud<pcl::PointXYZRGB>); 
		   pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalreg_right (new pcl::PointCloud<pcl::PointXYZRGB>); 
		   ///////////////////////Resize right point cloud size/////////////
		    pcl::copyPointCloud<pcl::PointNormal,pcl::PointXYZRGB>(*doncloud_right,*doncloud1_right );
		    printf("CurvatureRightsize=%d\n",doncloud1_right->points.size());

    		
			
			//////////////////////////line segmentation
			pcl::ModelCoefficients line; 
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
			pcl::SACSegmentation<pcl::PointXYZRGB> seg; 
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			vector < pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGB>::Ptr > > line_Segments_Clouds;
			vector < pcl::ModelCoefficients, Eigen::aligned_allocator <pcl::ModelCoefficients> > line_vector;
			vector<double> row;
			row.assign(6,0);
			vector< vector<double> > line_Segments_vertex;
			pcl::ModelCoefficients line2; 
			pcl::PointIndices::Ptr inliers2(new pcl::PointIndices); 
			pcl::SACSegmentation<pcl::PointXYZRGB> seg2; 
			pcl::ExtractIndices<pcl::PointXYZRGB> extract2;
			

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr line_project_to_xy (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud<pcl::PointXYZRGB,pcl::PointXYZRGB>(*doncloud1_right,*line_project_to_xy );
			int count=0;
			
			if(line_project_to_xy->points.size()>=10){
				for(int i=0;i<line_project_to_xy->points.size ();i++){
						line_project_to_xy->points[i].z=0;		//Preset z to 0 so that line ransac can be successful
			  	}		
			  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr line_project_to_xy_leave(new pcl::PointCloud<pcl::PointXYZRGB>) ;
			  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr line_project_to_xy_filtered(new pcl::PointCloud<pcl::PointXYZRGB>) ;
			  	line_project_to_xy_leave= line_project_to_xy;	
			  	printf("line_project_to_xy_leave size=%d\n",line_project_to_xy_leave->points.size());
			  	//////////if pointcloud size > 50, it will do ransac repeatedly
			  	while(line_project_to_xy_leave->points.size()>50){
					seg.setOptimizeCoefficients(true); 
					seg.setModelType(pcl::SACMODEL_LINE); 
					seg.setMethodType(pcl::SAC_RANSAC); 
					seg.setDistanceThreshold(0.2); 
					seg.setInputCloud(line_project_to_xy_leave); 
					seg.segment(*inliers, line); 
				    extract.setInputCloud (line_project_to_xy_leave);
				    extract.setIndices (inliers);
				    extract.setNegative (false);
				    extract.filter (*line_project_to_xy_filtered);
				    pointcloudResize(line_project_to_xy_filtered);
				    int regsize=line_project_to_xy_filtered->points.size ();
				    printf("OriginalSize=%d\n",line_project_to_xy_filtered->points.size ());
				    line_segment_filter(line_project_to_xy_filtered,line);
				    ///////////////////// If there is outlier, then ransac again to make sure that ransac will be more accurate.
				    if(line_project_to_xy_filtered->points.size ()<regsize){
				    	seg.setOptimizeCoefficients(true); 
						seg.setModelType(pcl::SACMODEL_LINE); 
						seg.setMethodType(pcl::SAC_RANSAC); 
						seg.setDistanceThreshold(0.2); 
						seg.setInputCloud(line_project_to_xy_filtered); 
						seg.segment(*inliers2, line2); 
					    extract.setInputCloud (line_project_to_xy_filtered);
					    extract.setIndices (inliers2);
					    extract.setNegative (false);
					    extract.filter (*line_project_to_xy_filtered);
				    }

				    extract.setInputCloud (line_project_to_xy_leave);
				    extract.setIndices (inliers);
				    extract.setNegative (true);
				    extract.filter (*line_project_to_xy_leave);
				    pointcloudResize(line_project_to_xy_leave);
				    printf("RansacSize=%d\n",line_project_to_xy_filtered->points.size ());
				    printf("LeftSize=%d\n",line_project_to_xy_leave->points.size ());
				    pcl::PointCloud<pcl::PointXYZRGB>::Ptr line_project_to_xy_reg(new pcl::PointCloud<pcl::PointXYZRGB>) ;
				    pcl::copyPointCloud(*line_project_to_xy_filtered,*line_project_to_xy_reg);

				    //If the nearest pointclouds of detected wall is too far , then ignore it.
				    if ( sqrt(pow(line_project_to_xy_reg->points[0].x,2)+pow(line_project_to_xy_reg->points[0].y,2)+pow(line_project_to_xy_reg->points[0].z,2))<5.0|| \
				    	sqrt(pow(line_project_to_xy_reg->points[line_project_to_xy_reg->points.size()-1].x,2)+pow(line_project_to_xy_reg->points[line_project_to_xy_reg->points.size()-1].y,2)\
				    	+pow(line_project_to_xy_reg->points[line_project_to_xy_reg->points.size()-1].z,2))<5.0){
				    	line_Segments_Clouds.push_back(line_project_to_xy_reg);
					    if(line_project_to_xy_filtered->points.size ()<regsize)
					    	line_vector.push_back(line2);
					    else
					    	line_vector.push_back(line);
					   
						count++;
						/*
						double color=0;
							if (count==0)
								color=0;
							if (count==1)
								color=0.5;
							if (count==2)
								color=1.0;
							if (count>2)
								color=1.0;
						for(int i=0;i<line_project_to_xy_filtered->points.size ();i++){
								
								bot_lcmgl_color3f(lcmgl_pointcloud,color, 0, 0 );
				    			bot_lcmgl_vertex3f(lcmgl_pointcloud, line_project_to_xy_filtered->points[i].x,line_project_to_xy_filtered->points[i].y, line_project_to_xy_filtered->points[i].z);	    
					  	}
					  	*/
				    }
				    
					
			  	}

			  	gettimeofday(&stop1,NULL);
				  duration1=(stop1.tv_sec-start1.tv_sec)+(stop1.tv_usec-start1.tv_usec)/1000000.0;
				  printf("Time ransac:\t%lf\n",duration1);
				  gettimeofday(&start1,NULL);
			  	line_Segments_vertex.assign(count,row);

			  	///////////////////Adjust the direction so the vector will be toward the front.
			  	for(int i=0;i<count;i++){
			  		double length_vertex1= pow(line_Segments_Clouds[i]->points[0].x,2) +pow(line_Segments_Clouds[i]->points[0].y,2)+\
			  			pow(line_Segments_Clouds[i]->points[0].z,2);
			  		double length_vertex2=pow(line_Segments_Clouds[i]->points[line_Segments_Clouds[i]->points.size()-1].x,2) +\
			  		pow(line_Segments_Clouds[i]->points[line_Segments_Clouds[i]->points.size()-1].y,2)+pow(line_Segments_Clouds[i]->points[line_Segments_Clouds[i]->points.size()-1].z,2);
			  		if(length_vertex1<=length_vertex2){
			  			line_Segments_vertex[i][0]=line_Segments_Clouds[i]->points[0].x;
				  		line_Segments_vertex[i][1]=line_Segments_Clouds[i]->points[0].y;
				  		line_Segments_vertex[i][2]=line_Segments_Clouds[i]->points[0].z;
				  		line_Segments_vertex[i][3]=line_Segments_Clouds[i]->points[line_Segments_Clouds[i]->points.size()-1].x;
				  		line_Segments_vertex[i][4]=line_Segments_Clouds[i]->points[line_Segments_Clouds[i]->points.size()-1].y;
				  		line_Segments_vertex[i][5]=line_Segments_Clouds[i]->points[line_Segments_Clouds[i]->points.size()-1].z;
			  		}
			  		else{
			  			line_Segments_vertex[i][3]=line_Segments_Clouds[i]->points[0].x;
				  		line_Segments_vertex[i][4]=line_Segments_Clouds[i]->points[0].y;
				  		line_Segments_vertex[i][5]=line_Segments_Clouds[i]->points[0].z;
				  		line_Segments_vertex[i][0]=line_Segments_Clouds[i]->points[line_Segments_Clouds[i]->points.size()-1].x;
				  		line_Segments_vertex[i][1]=line_Segments_Clouds[i]->points[line_Segments_Clouds[i]->points.size()-1].y;
				  		line_Segments_vertex[i][2]=line_Segments_Clouds[i]->points[line_Segments_Clouds[i]->points.size()-1].z;
			  		}
			  		printf("VERTEX:(%g,%g,%g)(%g,%g,%g)\n",line_Segments_vertex[i][0],line_Segments_vertex[i][1],\
			  			line_Segments_vertex[i][2],line_Segments_vertex[i][3],line_Segments_vertex[i][4],\
			  			line_Segments_vertex[i][5]);
			  	}

			  	/*
			  	//Calculate the average y to know the segment is on the right or on the left
			  	vector<double> cloud_average(count, 0);  
			  	printf("averageY: ");
			  	for(int i=0;i<count;i++){
			  		double sum=0;
			  		for(int j=0;j<line_Segments_Clouds[i]->points.size();j++)
			  			sum += line_Segments_Clouds[i]->points[j].y;
			  		sum/=line_Segments_Clouds[i]->points.size();
			  		cloud_average[i]=sum;
			  		printf("%g ",sum);
			  	}
			  	printf("\n");
			  	*/
			  	int roadtype=0;
			  	int corner=0; //left:1, right:2
			  	double corner_index_1=-1;
				double corner_index_2=-2;
				//Check whether it is a intersection or one way or death way
		  		for(int i=0;i<count;i++){
		  			//printf("X, -Y, Z:\t%g %g %g %g %g %g\n",line_vector[i].values[0],line_vector[i].values[1],line_vector[i].values[2],line_vector[i].values[3],line_vector[i].values[4],line_vector[i].values[5]);
		  			for(int j=i+1;j<count;j++){
		  				double Dot_Product=line_vector[i].values[3]*line_vector[j].values[3]+line_vector[i].values[4]*line_vector[j].values[4]+line_vector[i].values[5]*line_vector[j].values[5];
		  				Dot_Product=fabs(Dot_Product);
		  				//Check whether it is a intersection or one way
		  				if(Dot_Product<0.85){
		  					roadtype=1;
		  					if(sqrt(pow(line_Segments_vertex[i][0]-line_Segments_vertex[j][0] ,2)+pow(line_Segments_vertex[i][1]-line_Segments_vertex[j][1] ,2)+\
		  					pow(line_Segments_vertex[i][2]-line_Segments_vertex[j][2] ,2))<1){
		  						corner_index_1=i;
		  						corner_index_2=j;
		  					}
		  					//Check whether it is a death way
		  					if(sqrt(pow(line_Segments_vertex[i][3]-line_Segments_vertex[j][3] ,2)+pow(line_Segments_vertex[i][4]-line_Segments_vertex[j][4] ,2)+\
		  					pow(line_Segments_vertex[i][5]-line_Segments_vertex[j][5] ,2))<1){
		  						roadtype=2;
		  						break;
		  					}
		  				}
		  				printf("Inner dot\t%g\n",Dot_Product);
		  			}
		  			if(roadtype==2)
		  				break;
		  		}
		  		gettimeofday(&stop1,NULL);
				duration1=(stop1.tv_sec-start1.tv_sec)+(stop1.tv_usec-start1.tv_usec)/1000000.0;
				printf("Time Intersection detection:\t%lf\n",duration1);
				gettimeofday(&start1,NULL);

		  		/////////////////Produce segmentlist
		  		segment->number=count;

		  		/////////////////Roadtype1:  intersection
				if(roadtype==1){
					segment->road_classify=1;
					printf("This is a intersection\n");
												
				}
				/////////////////Roadtype2:  One way
				else if (roadtype==0){
					segment->road_classify=0;
				 	
				}
				/////////////////Roadtype3: Death way
				else if (roadtype==2){
					segment->road_classify=2;
					printf("This is a death way.\n");
				}


				kinect_segment_t *segmentlist =  (kinect_segment_t*)malloc(count*sizeof(kinect_segment_t));
				segment->segmentlist=segmentlist;
				
				for(int i=0;i<count;i++){
					printf("Original wall direction:\t%g %g %g %g\n",line_Segments_vertex[i][0],line_Segments_vertex[i][1],line_Segments_vertex[i][3],line_Segments_vertex[i][4]);
					segment->segmentlist[i].line[0].u=line_Segments_vertex[i][0];
					segment->segmentlist[i].line[0].v=line_Segments_vertex[i][1];

					////Recorrect wall direction
					if((line_Segments_vertex[i][3]-line_Segments_vertex[i][0])*line_vector[i].values[3]+\
					(line_Segments_vertex[i][4]-line_Segments_vertex[i][1])*line_vector[i].values[4]>0){
						segment->segmentlist[i].line[1].u=line_Segments_vertex[i][0]+line_vector[i].values[3];
						segment->segmentlist[i].line[1].v=line_Segments_vertex[i][1]+line_vector[i].values[4];
					}
					else{
						segment->segmentlist[i].line[1].u=line_Segments_vertex[i][0]-line_vector[i].values[3];
						segment->segmentlist[i].line[1].v=line_Segments_vertex[i][1]-line_vector[i].values[4];
					}
							//Recorrect direction if there is a deathway
					if (roadtype==2){
						segment->segmentlist[i].line[0].u=1.0;
						segment->segmentlist[i].line[0].v=-1.0;
						segment->segmentlist[i].line[1].u=1.0+1.0/sqrt(2);
						segment->segmentlist[i].line[1].v=-1.0+1.0/sqrt(2);
						final_right_direction[0]=1;final_right_direction[1]=1;final_right_distance=sqrt(2);
						printf("Death way:\t%g %g %g %g\n",segment->segmentlist[i].line[0].u,segment->segmentlist[i].line[0].v,segmentlist[i].line[1].u-segmentlist[i].line[0].u,segmentlist[i].line[1].v-segmentlist[i].line[0].v);
						
					}					
							
					//Define the right way or left way
					segment->segmentlist[i].side=1;
					if (roadtype!=2){
						if(segment->segmentlist[i].line[0].v<0){								
							segment->segmentlist[i].side=1;			//right:1
							printf("right wall:\t%g %g %g %g\n",segment->segmentlist[i].line[0].u,segment->segmentlist[i].line[0].v,segmentlist[i].line[1].u-segmentlist[i].line[0].u,segmentlist[i].line[1].v-segmentlist[i].line[0].v);
							double reg=1.0/sqrt(pow(segment->segmentlist[i].line[0].u,2)+pow(segment->segmentlist[i].line[0].v,2));
							final_right_distance+=reg;
							final_right_direction[0]+=reg*(segmentlist[i].line[1].u-segmentlist[i].line[0].u);
							final_right_direction[1]+=reg*(segmentlist[i].line[1].v-segmentlist[i].line[0].v);
						}
						if(segment->segmentlist[i].line[0].v>0){
							segment->segmentlist[i].side=0;			//left:0
							printf("left wall:\t%g %g %g %g\n",segment->segmentlist[i].line[0].u,segment->segmentlist[i].line[0].v,segmentlist[i].line[1].u-segmentlist[i].line[0].u,segmentlist[i].line[1].v-segmentlist[i].line[0].v);
							double reg=1.0/sqrt(pow(segment->segmentlist[i].line[0].u,2)+pow(segment->segmentlist[i].line[0].v,2));
							final_left_distance+=reg;
							final_left_direction[0]+=reg*(segmentlist[i].line[1].u-segmentlist[i].line[0].u);
							final_left_direction[1]+=reg*(segmentlist[i].line[1].v-segmentlist[i].line[0].v);
						}
					}
					
				}
				//Visualize forward direction
				if (roadtype==0){
					if(final_right_distance<=0){
						final_right_distance=final_left_distance;
						final_right_direction[0]=final_left_direction[0]/final_left_distance;
						final_right_direction[1]=final_left_direction[1]/final_left_distance;

					}					
				}
				if (roadtype==1){
					final_right_distance+=final_left_distance;
					final_right_direction[0]+=final_left_direction[0]*final_left_distance;
					final_right_direction[1]+=final_left_direction[1]*final_left_distance;
					final_right_direction[0]=final_right_direction[0]/final_right_distance;
					final_right_direction[1]=final_right_direction[1]/final_right_distance;
				}
				
			}


			//Draw the depth Image
			
    		for(int i=0;i<cloud_raw->points.size ();i++){
				bot_lcmgl_color3f(lcmgl_pointcloud, cloud_raw->points[i].r/255.0, cloud_raw->points[i].g/255.0, cloud_raw->points[i].b/255.0);
    			bot_lcmgl_vertex3f(lcmgl_pointcloud, cloud_raw->points[i].x, cloud_raw->points[i].y, cloud_raw->points[i].z);	    
		   }
			if (count>0){
				kinect_segmentlist_v2_t_publish(this->lcm_, "Segmentlist", this->segment);
				Eigen::Vector3d start_pt_left;
			   	Eigen::Vector3d stop_pt_left;				   
			   	start_pt_left << 0.0,0.0,0.0;			   
			   	stop_pt_left << final_right_direction[0],final_right_direction[1],0.0;
			    rv::draw_line_lcmgl(lcmgl_pointcloud, start_pt_left, stop_pt_left,1,0,0);
			    rv::draw_line_lcmgl(lcmgl_pointcloud, start_pt_left, stop_pt_left,1,0,0);
			}
			

			/*
			//Draw the found wall
			if(doncloud1_right->points.size()>=10){
				for(int i=0;i<doncloud1_right->points.size ();i++){
						bot_lcmgl_color3f(lcmgl_pointcloud, 0,  1, 0);
						//bot_lcmgl_color3f(lcmgl_pointcloud, 0,  doncloud1_right->points[i].g/255.0, 0);
		    			bot_lcmgl_vertex3f(lcmgl_pointcloud, doncloud1_right->points[i].x,doncloud1_right->points[i].y, doncloud1_right->points[i].z);	    
			  	}			  
			}
			*/
	   
		   
		   for(int i=0;i<count;i++){
		   		Eigen::Vector3d start_pt_left;
			   	Eigen::Vector3d stop_pt_left;				   
			   	start_pt_left << line_vector[i].values[0],line_vector[i].values[1],line_vector[i].values[2];			   
			   	stop_pt_left << line_vector[i].values[0]+4*line_vector[i].values[3],line_vector[i].values[1]+4*line_vector[i].values[4],line_vector[i].values[2]+4*line_vector[i].values[5];
			    rv::draw_line_lcmgl(lcmgl_pointcloud, start_pt_left, stop_pt_left,0,0,1);
			    //rv::draw_line_lcmgl(lcmgl_pointcloud, start_pt_left, stop_pt_left);
		   }
	  	
			bot_lcmgl_end(lcmgl_pointcloud);
    		bot_lcmgl_switch_buffer(lcmgl_pointcloud);
    		////////////////////////////Free memory
    		
    		gettimeofday(&stop1,NULL);
				  duration1=(stop1.tv_sec-start1.tv_sec)+(stop1.tv_usec-start1.tv_usec)/1000000.0;
				  printf("Time draw:\t%lf\n",duration1);
				  gettimeofday(&start1,NULL);

    	}
    	gettimeofday(&stop,NULL);
		duration=(stop.tv_sec-start.tv_sec)+(stop.tv_usec-start.tv_usec)/1000000.0;
		printf("CPU Time:\t%lf\n",duration);
    	delete rgb_buf_;	

		/////////////////////////////////////Cloud segmentation/////////////////////////////////////////////////////
		return;
		

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
	int decimate = 10.0;
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


