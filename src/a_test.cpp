#include "republisher/republisher.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "insole_msgs/InsoleSensorStamped.h"
#include <SimTKcommon/Orientation.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <SimTKcommon/internal/Rotation.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "rep");
	ros::NodeHandle nh("~");
	double rate;
	nh.param<double>("rate", rate, 100);
	auto r = ros::Rate(rate);
//	while(ros::ok())
	{
		r.sleep();
		ros::spinOnce();
	}
	
    	double xRad = SimTK::convertDegreesToRadians(0);
    	double yRad = SimTK::convertDegreesToRadians(0);
    	double zRad = SimTK::convertDegreesToRadians(0);
	auto R_GoGi = SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, xRad,
                      SimTK::XAxis, yRad, SimTK::YAxis, zRad, SimTK::ZAxis);
    ROS_WARN_STREAM("ground orientation matrix:\n" << R_GoGi);
	SimTK::Quaternion q0{0.5, 0.5, 0.5, 0.5};
	SimTK::Rotation R = SimTK::Rotation(q0);
	ROS_INFO_STREAM(R);
	//to quaternion
	SimTK::Quaternion qz= R.convertRotationToQuaternion();
	ROS_INFO_STREAM("qz as Quaternion" << qz);
	ROS_INFO_STREAM("byebye");

	return 0;
}
