#include "insole_msgs/InsoleSensorStamped.h"
#include "republisher/republisher.h"
#include "ros/init.h"

int main(int argvc, char **argv)
{
	ros::init(argvc, argv, "rep");
	Republisher<insole_msgs::InsoleSensorStamped> rep;
	ros::spin();
	return 0;
}
