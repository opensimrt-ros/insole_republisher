#include "geometry_msgs/WrenchStamped.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "republisher/republisher.h"
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

			filterParam.numSignals = 3+2; // +2 for cop filtering
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

struct PublicationBuffer
{
	geometry_msgs::WrenchStamped w_oversampled;
	geometry_msgs::WrenchStamped w_filtered;
	geometry_msgs::TransformStamped t_oversampled;
	geometry_msgs::TransformStamped t_filtered;
};


class SyncRepublisher
{
	public:
		SyncRepublisher()
		{
			nh.param<double>("rate", rate, 100);
			r = new ros::Rate(rate);
			wfilter = new RosOpenSimRTFilter(nh);
			wrench_publisher = nh.advertise<geometry_msgs::WrenchStamped>("wrench_oversampled",10);
			wrench_publisher_f = nh.advertise<geometry_msgs::WrenchStamped>("wrench_filtered",10);
			ik_sub = nh.subscribe("ik",1,&SyncRepublisher::callback, this);
		}
		void run()
		{
			std::optional<insole_msgs::InsoleSensorStamped> el_o = rep.get_latest();
			if (!el_o)
			{
				r->sleep();
				ros::spinOnce();
				return;
			}
			auto el = el_o.value();
			if (!at.initial_time)
				at.set_initial_time(el.header);
			double time = at.now(el.header);
			ROS_DEBUG_STREAM(time);
			SimTK::Vector x(5);
			auto w = geometry_msgs::WrenchStamped();
			w.header = el.header;
			w.wrench = el.wrench;

			///////wrench_publisher.publish(w);
			auto st = geometry_msgs::TransformStamped();
			st.header = el.header;
			st.header.frame_id = el.ts.header.frame_id;
			st.transform = el.ts.transform;
			st.child_frame_id = el.ts.child_frame_id+"_oversampled";
			/////////tf.sendTransform(st);
			pb.w_oversampled = w;
			pb.t_oversampled = st;
			ROS_DEBUG_STREAM("Sent oversampled values to buffer.");
			//modify w to its filtered version
			if (wfilter->publish_filtered)
			{
				ROS_DEBUG_STREAM("Calculating filtered values.");
				x[0] = w.wrench.force.x;
				x[1] = w.wrench.force.y;
				x[2] = w.wrench.force.z;
				x[3] = st.transform.translation.x;
				x[4] = st.transform.translation.y;
				auto x_filtered = wfilter->filter(time, x);
				if (x_filtered.isValid)
				{
					ROS_DEBUG_STREAM("Filter is valid");
					auto w_filtered =w;
					w_filtered.wrench.force.x = x_filtered.x[0];
					w_filtered.wrench.force.y = x_filtered.x[1];
					w_filtered.wrench.force.z = x_filtered.x[2];
					ROS_DEBUG_STREAM(x <<"  "<<x_filtered.x);
					///////wrench_publisher_f.publish(w);
					auto st_filtered = st;
					st_filtered.transform.translation.x = x_filtered.x[3];
					st_filtered.transform.translation.y = x_filtered.x[4];
					st_filtered.child_frame_id = el.ts.child_frame_id + "_filtered";
					///////////tf.sendTransform(st_filtered);
					pb.t_filtered = st_filtered;
					pb.w_filtered = w_filtered;
					ROS_DEBUG_STREAM("Sent values to buffer.");
				}
			}
			r->sleep();
		}
	private:
		ros::NodeHandle nh{"~"};
		double rate;
		ros::Rate* r;
		Republisher<insole_msgs::InsoleSensorStamped> rep;
		ros::Publisher wrench_publisher; 
		ros::Publisher wrench_publisher_f;
		ros::Subscriber ik_sub; 
		tf::TransformBroadcaster tf;
		PublicationBuffer pb;
		RosOpenSimRTFilter* wfilter;
		AppropriateTime at;
		std_msgs::Header::_stamp_type previous_time;
		void callback(opensimrt_msgs::CommonTimed m)
		{
			//what we want here is to have the time from the ik so we can use exacttime policy. I am still not sure this is the issue, but I will try it out. Maybe what we really need is a time synchronizer on the publisher of ID, or maybe we need both, I don't know yet.
			auto ik_header_stamp = m.header.stamp; // not sure if this works.
			if (ik_header_stamp < previous_time)
				ROS_FATAL_STREAM("MESSAGES ARE OUT OF ORDER");
			//update stamps
			pb.t_filtered.header.stamp = ik_header_stamp;
			pb.t_oversampled.header.stamp = ik_header_stamp;
			pb.w_filtered.header.stamp = ik_header_stamp;
			pb.w_oversampled.header.stamp = ik_header_stamp;

			ROS_DEBUG_STREAM(pb.t_filtered << pb.t_oversampled << pb.w_filtered << pb.w_oversampled);
			wrench_publisher.publish(pb.w_oversampled);
			if (pb.t_oversampled.child_frame_id != "")
			{
				tf.sendTransform(pb.t_oversampled);
			}
			if (pb.t_filtered.child_frame_id != "")
			{
				tf.sendTransform(pb.t_filtered);
			}
			wrench_publisher_f.publish(pb.w_filtered);
			previous_time = ik_header_stamp;
		};

};


int main(int argvc, char **argv)
{
	ros::init(argvc, argv, "rep");
	SyncRepublisher sr;
	while(ros::ok())
	{
		sr.run();
		ros::spinOnce();
	}

	return 0;
}
