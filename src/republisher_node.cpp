#include "republisher/republisher.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "insole_msgs/InsoleSensorStamped.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rep");
	ros::NodeHandle nh("~");
	double rate;
	nh.param<double>("rate", rate, 100);
	auto r = ros::Rate(rate);
	Republisher<insole_msgs::InsoleSensorStamped> rep;
	while(ros::ok())
	{
		rep.publish_one();
		r.sleep();
		ros::spinOnce();
	}

	return 0;
}
