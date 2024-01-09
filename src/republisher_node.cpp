#include "republisher/insole_republisher.h"
#include "ros/init.h"
#include "ros/node_handle.h"

//filter

int main(int argvc, char **argv)
{
	ros::init(argvc, argv, "rep");
	double rate;
	ros::NodeHandle nh("~");
	nh.param<double>("rate", rate, 100);
	ros::Rate r = ros::Rate(rate);
	InsoleRepublisherWithTimeCorrection rep;
	while(ros::ok())
	{
		rep.run();
		r.sleep();
		ros::spinOnce();
	}

	return 0;
}
