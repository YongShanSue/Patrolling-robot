#include <cstdlib>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core/pose_t.hpp"
#include "lcmtypes/april_tags/caffe_class_t.hpp"
#include "lcmtypes/april_tags/caffe_class_array_t.hpp"
#include "lcmtypes/april_tags/quad_proposal_t.hpp"
#include <lcmtypes/april_tags/tessocr_string_t.hpp>
#include <lcmtypes/kinect/segmentlist_t.hpp>


#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <map>

#include <duckietown_msgs/Twist2DStamped.h>
#include <duckietown_msgs/BoolStamped.h>
#include <duckietown_msgs/Rect.h>
#include <duckietown_msgs/SegmentList.h>

using namespace std;

class LCM2ROS
{
	public:
		// class announcement 
		LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_, std::string &veh_);
		~LCM2ROS() {}
	private:
		// lcm handler 
		boost::shared_ptr<lcm::LCM> lcm_;
		// ros handler 
		ros::NodeHandle nh_;
		// vehicle name 
        std::string veh_; 

		// caffe prediction callback 
		void segmentlistHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const kinect::segmentlist_t* msg);



		// class prediction publisher 
		ros::Publisher pub_segment;

		// ros node handler 
		ros::NodeHandle* rosnode;
};

// class LCM2ROS initialization
LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_, std::string &veh_): lcm_(lcm_),nh_(nh_),veh_(veh_) 
{
	
	// topics 
	std::stringstream topic_pub_carcmd;
	
	// lcm segmentlist subsciber 
	lcm_->subscribe("Segmentlist", &LCM2ROS::segmentlistHandler, this);


	topic_pub_carcmd << "/" << veh_ << "/L2R_Segment";
	pub_segment= nh_.advertise<duckietown_msgs::SegmentList>(topic_pub_carcmd.str(),10); 


	rosnode = new ros::NodeHandle();
	//ros::spin();
}

//caffe prediction callback 
void LCM2ROS::segmentlistHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const kinect::segmentlist_t* msg)
{
	duckietown_msgs::SegmentList segments_msg ;
	duckietown_msgs::Segment left_segment_msg ;
	duckietown_msgs::Segment right_segment_msg ;
	left_segment_msg.color=left_segment_msg.YELLOW;
	left_segment_msg.pixels_normalized[0].x=msg->left_normalized_line[0].u;
	left_segment_msg.pixels_normalized[0].y=msg->left_normalized_line[0].v;
	left_segment_msg.pixels_normalized[1].x=msg->left_normalized_line[1].u;
	left_segment_msg.pixels_normalized[1].y=msg->left_normalized_line[1].v;
	right_segment_msg.color=right_segment_msg.WHITE;
	right_segment_msg.pixels_normalized[0].x=msg->right_normalized_line[0].u;
	right_segment_msg.pixels_normalized[0].y=msg->right_normalized_line[0].v;
	right_segment_msg.pixels_normalized[1].x=msg->right_normalized_line[1].u;
	right_segment_msg.pixels_normalized[1].y=msg->right_normalized_line[1].v;
	//left_segment_msg.normal.x=
	//left_segment_msg.normal.y=
	segments_msg.segments.push_back(left_segment_msg);
	segments_msg.segments.push_back(right_segment_msg);
	
    pub_segment.publish(segments_msg);
    //segment_msg.segments
    printf("Get lcmmessage\n");
    printf("A number:	%lf\n",msg->left_normalized_line[0].u);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "L2R_text_navigation", ros::init_options::NoSigintHandler);
	boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);

	if(!lcm->good())
	{
		std::cerr << "ERROR: lcm is not good()" << std::endl;
	}	
	ros::NodeHandle nh("~");
	std::string veh;

	LCM2ROS handler_turn(lcm,nh,veh);
	cout << "lcm2ros translator ready";

	while(0 == lcm->handle());
	return 0;
}
