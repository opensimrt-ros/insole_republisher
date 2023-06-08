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
class RosOpenSimRTFilter
{
	public:
		OpenSimRT::LowPassSmoothFilter::Parameters filterParam;
		bool publish_filtered;	
		double cutoffFreq;
		int splineOrder, memory, delay;
		OpenSimRT::LowPassSmoothFilter * ofilter;
		RosOpenSimRTFilter(ros::NodeHandle nh)
		{
			nh.param<bool>("filter_output",publish_filtered, true);
			nh.param<double>("cutoff_freq", cutoffFreq, 0.0);
			nh.param<int>("memory", memory, 0);
			nh.param<int>("spline_order", splineOrder, 0);
			nh.param<int>("delay", delay, 0);

			filterParam.numSignals = 3;
			filterParam.memory = memory;
			filterParam.delay = delay;
			filterParam.cutoffFrequency = cutoffFreq;
			filterParam.splineOrder = splineOrder;
			filterParam.calculateDerivatives = true;
			ofilter = new OpenSimRT::LowPassSmoothFilter(filterParam);
		}
		OpenSimRT::LowPassSmoothFilter::Output filter(double t, SimTK::Vector v)
		{
			return ofilter->filter({t,v});
		}
};

class AppropriateTime
{
	public:
		std::optional<ros::Time> initial_time;
		void set_initial_time(std_msgs::Header h)
		{
			initial_time = h.stamp;				

		}
		double now(std_msgs::Header h)
		{
			ros::Duration d = h.stamp - initial_time.value();
			return d.toSec();
		}
};

int main(int argvc, char **argv)
{
	ros::init(argvc, argv, "rep");
	ros::NodeHandle nh("~");
	double rate;
	nh.param<double>("rate", rate, 100);
	auto r = ros::Rate(rate);
	Republisher<insole_msgs::InsoleSensorStamped> rep;
	ros::Publisher wrench_publisher = nh.advertise<geometry_msgs::WrenchStamped>("wrench_oversampled",1);
	ros::Publisher wrench_publisher_f = nh.advertise<geometry_msgs::WrenchStamped>("wrench_filtered",1);
	tf::TransformBroadcaster tf;

	RosOpenSimRTFilter* wfilter = new RosOpenSimRTFilter(nh);
	AppropriateTime at;
	while(ros::ok())
	{
		std::optional<insole_msgs::InsoleSensorStamped> el_o = rep.get_latest();
		if (!el_o)
		{
			r.sleep();
			ros::spinOnce();
			continue;
		}
		auto el = el_o.value();
		if (!at.initial_time)
			at.set_initial_time(el.header);
		double time = at.now(el.header);
		ROS_DEBUG_STREAM(time);
		SimTK::Vector x(3);
		auto w = geometry_msgs::WrenchStamped();
		w.header = el.header;
		w.wrench = el.wrench;

		wrench_publisher.publish(w);
		auto st = geometry_msgs::TransformStamped();
		st.header = el.header;
		st.header.frame_id = el.ts.header.frame_id;
		st.transform = el.ts.transform;
		st.child_frame_id = el.ts.child_frame_id+"_oversampled";
		tf.sendTransform(st);
		//modify w to its filtered version
		if (wfilter->publish_filtered)
		{
			x[0] = w.wrench.force.x;
			x[1] = w.wrench.force.y;
			x[2] = w.wrench.force.z;
			auto x_filtered = wfilter->filter(time, x);
			if (x_filtered.isValid)
			{
				w.wrench.force.x = x_filtered.x[0];
				w.wrench.force.y = x_filtered.x[1];
				w.wrench.force.z = x_filtered.x[2];
				ROS_DEBUG_STREAM(x <<"  "<<x_filtered.x);
				wrench_publisher_f.publish(w);
			}
		}
		r.sleep();
		ros::spinOnce();
	}

	return 0;
}
