#include "geometry_msgs/WrenchStamped.h"
#include "republisher/republisher.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "insole_msgs/InsoleSensorStamped.h"
#include "ros/time.h"
#include "tf/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "SignalProcessing.h"
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Vec.h>

//filter
#include "republisher/filter.h"

int main(int argvc, char **argv)
{
	ros::init(argvc, argv, "rep");
	Republisher<insole_msgs::InsoleSensorStamped> rep;
	while(ros::ok())
	{
		rep.run();
		rep.r->sleep();
		ros::spinOnce();
	}

	return 0;
}
