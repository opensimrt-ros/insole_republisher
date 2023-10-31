#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/WrenchStamped.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "republisher/republisher.h"
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "insole_msgs/InsoleSensorStamped.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include "tf/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "SignalProcessing.h"
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Vec.h>

//filter
#include "republisher/filter.h"


template <typename TriggerMessageType>
class SyncRepublisher
{
	public:
		SyncRepublisher()
		{
			ik_sub = nh.subscribe("ik",1,&SyncRepublisher::callback, this);
			ROS_INFO_STREAM("git heer");
		}
		Republisher<insole_msgs::InsoleSensorStamped> rep;
	private:
		ros::NodeHandle nh{"~"};
		ros::Subscriber ik_sub; 
		std_msgs::Header::_stamp_type previous_time;
		void callback(TriggerMessageType m)
		{
			//what we want here is to have the time from the ik so we can use exacttime policy. I am still not sure this is the issue, but I will try it out. Maybe what we really need is a time synchronizer on the publisher of ID, or maybe we need both, I don't know yet.
			auto ik_header_stamp = m.header.stamp; // not sure if this works.
			if (ik_header_stamp < previous_time)
				ROS_FATAL_STREAM("MESSAGES ARE OUT OF ORDER");
			//backstamping with delay
			ROS_DEBUG_STREAM("backstamping with delay" << rep.delay);
			ik_header_stamp -= rep.delay;

			//update stamps
			rep.pb.t_filtered.header.stamp = ik_header_stamp;
			rep.pb.t_oversampled.header.stamp = ik_header_stamp;
			rep.pb.w_filtered.header.stamp = ik_header_stamp;
			rep.pb.w_oversampled.header.stamp = ik_header_stamp;

			previous_time = ik_header_stamp;
		};

};


int main(int argvc, char **argv)
{
	ros::init(argvc, argv, "rep");
	if( false && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) //true to debug
	{
		ros::console::notifyLoggerLevelsChanged();
	}
	//TODO:reads the message trigger type and instantiate the appropriate type of class to sync to. 
	ROS_INFO_STREAM("get here");
		SyncRepublisher<opensimrt_msgs::CommonTimed> sr;
	ROS_INFO_STREAM("also here");
	while(ros::ok())
	{
		sr.rep.run();
		ros::spinOnce();
		sr.rep.r->sleep(); //make sure that every branch has the sleep.
	}

	return 0;
}
