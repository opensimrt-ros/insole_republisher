#include "dynamic_reconfigure/server.h"
#include "insole_msgs/InsoleSensorStamped.h"
#include "republisher/republisher.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include <republisher/delayConfig.h>


int main(int argvc, char **argv)
{
	ros::init(argvc, argv, "rep");
	ros::NodeHandle nh("~");
	Republisher<insole_msgs::InsoleSensorStamped> rep;
	dynamic_reconfigure::Server<insole_republisher::delayConfig> delay_server_(nh);
	dynamic_reconfigure::Server<insole_republisher::delayConfig>::CallbackType f;
	f = boost::bind(&Republisher<insole_msgs::InsoleSensorStamped>::reconfigure_delay_callback, &rep, _1, _2);
	delay_server_.setCallback(f);

	ros::spin();
	return 0;
}
