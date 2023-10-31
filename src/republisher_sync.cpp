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

struct PublicationBuffer
{
	geometry_msgs::WrenchStamped w_oversampled;
	geometry_msgs::WrenchStamped w_filtered;
	geometry_msgs::TransformStamped t_oversampled;
	geometry_msgs::TransformStamped t_filtered;
};

template <typename TriggerMessageType>
class SyncRepublisher
{
	public:
		ros::Rate* r;
		double old_time;
		ros::Duration delay;
		double delay_seconds;
		bool debug_publish_zero_cop, debug_publish_fixed_force;
		SyncRepublisher()
		{
			nh.param<double>("rate", rate, 100);
			nh.param<double>("wrench_delay", delay_seconds, 0); 
			nh.param<bool>("debug_publish_zero_cop", debug_publish_zero_cop, false);
			nh.param<bool>("debug_publish_fixed_force", debug_publish_fixed_force, false);
			delay.fromSec(delay_seconds);
			r = new ros::Rate(rate);
			wfilter = new RosOpenSimRTFilter(nh, 6);
			wrench_publisher = nh.advertise<geometry_msgs::WrenchStamped>("wrench_oversampled",1);
			wrench_publisher_f = nh.advertise<geometry_msgs::WrenchStamped>("wrench_filtered",1);
			ik_sub = nh.subscribe("ik",1,&SyncRepublisher::callback, this);
		}
		void run()
		{
			std::optional<insole_msgs::InsoleSensorStamped> el_o = rep.get_latest();
			if (!el_o)
			{
				return;
			}
			auto el = el_o.value();
			if (!at.initial_time)
			{
				at.set_initial_time(el.header,delay);
				old_time = at.now(el.header);
			}

			double time = at.now(el.header);
			if (time<=old_time)
			{
				ROS_ERROR("time is smaller than previous time. this is a problem");
				return;
			}
			ROS_DEBUG_STREAM(time);
			auto w = geometry_msgs::WrenchStamped();
			w.header = el.header;
			w.wrench = el.wrench;
			w.header.frame_id = el.header.frame_id + "_oversampled";
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
				SimTK::Vector x(6);
				x[0] = w.wrench.force.x;
				x[1] = w.wrench.force.y;
				x[2] = w.wrench.force.z;
				x[3] = st.transform.translation.x;
				x[4] = st.transform.translation.y;
				x[5] = st.transform.translation.z;
				auto x_filtered = wfilter->filter(time, x);
				//makes sure i always have a valid header even when the filter is not yet ready.
				auto st_filtered = st;
				st_filtered.child_frame_id = el.ts.child_frame_id + "_filtered";
				if (x_filtered.isValid)
				{
					ROS_DEBUG_STREAM("Filter is valid");
					auto w_filtered =w;
					w_filtered.header.frame_id = st_filtered.child_frame_id; //this is the correct frame for filtered, right?
					if (debug_publish_fixed_force)
					{
						w_filtered.wrench.force = geometry_msgs::Vector3();
						w_filtered.wrench.force.y = 100;
					}
					else
					{
						w_filtered.wrench.force.x = x_filtered.x[0];
						w_filtered.wrench.force.y = x_filtered.x[1];
						w_filtered.wrench.force.z = x_filtered.x[2];
					}
					ROS_DEBUG_STREAM(x <<"  "<<x_filtered.x);
					///////wrench_publisher_f.publish(w);
					if (debug_publish_zero_cop)
					{
						st_filtered.transform.translation = geometry_msgs::Vector3();
					}
					else
					{
						st_filtered.transform.translation.x = x_filtered.x[3];
						st_filtered.transform.translation.y = x_filtered.x[4];
						st_filtered.transform.translation.z = x_filtered.x[5];
					}
					///////////tf.sendTransform(st_filtered);
					pb.t_filtered = st_filtered;
					pb.w_filtered = w_filtered;
					ROS_DEBUG_STREAM("Sent values to 'buffer' (it's not really a buffer. there is one sample there...).");
				}
			}
			old_time = time;
		}
	private:
		ros::NodeHandle nh{"~"};
		double rate;
		Republisher<insole_msgs::InsoleSensorStamped> rep;
		ros::Publisher wrench_publisher; 
		ros::Publisher wrench_publisher_f;
		ros::Subscriber ik_sub; 
		tf::TransformBroadcaster tf;
		PublicationBuffer pb;
		RosOpenSimRTFilter* wfilter;
		AppropriateTime at;
		std_msgs::Header::_stamp_type previous_time;
		void callback(TriggerMessageType m)
		{
			//what we want here is to have the time from the ik so we can use exacttime policy. I am still not sure this is the issue, but I will try it out. Maybe what we really need is a time synchronizer on the publisher of ID, or maybe we need both, I don't know yet.
			auto ik_header_stamp = m.header.stamp; // not sure if this works.
			if (ik_header_stamp < previous_time)
				ROS_FATAL_STREAM("MESSAGES ARE OUT OF ORDER");
			//backstamping with delay
			ROS_DEBUG_STREAM("backstamping with delay" << delay);
			ik_header_stamp -= delay;

			//update stamps
			pb.t_filtered.header.stamp = ik_header_stamp;
			pb.t_oversampled.header.stamp = ik_header_stamp;
			pb.w_filtered.header.stamp = ik_header_stamp;
			pb.w_oversampled.header.stamp = ik_header_stamp;

			ROS_DEBUG_STREAM(pb.t_filtered << pb.t_oversampled << pb.w_filtered << pb.w_oversampled);
			wrench_publisher.publish(pb.w_oversampled);
			// check if the tf trasnformaitons are complete
			if (pb.t_oversampled.header.frame_id == "")
			{
				ROS_ERROR("t_oversampled has no source frame_id");
			}
			else
			{
				if (pb.t_oversampled.child_frame_id != "")
				{
					tf.sendTransform(pb.t_oversampled);
				}
				else
				{
					ROS_ERROR("oversampled has no child_frame_id!");
				}
			}
			if (pb.t_filtered.header.frame_id == "")
			{
				ROS_ERROR("t_filtered has no source frame_id");
			}
			else
			{
				if (pb.t_filtered.child_frame_id != "")
				{
					tf.sendTransform(pb.t_filtered);
					//we will only publish if everything is okay with the transform too. 
					wrench_publisher_f.publish(pb.w_filtered);
				}
				else
				{
					ROS_ERROR("filtered has no child_frame_id");
				}
			}
			previous_time = ik_header_stamp;
		};

};


int main(int argvc, char **argv)
{
	ros::init(argvc, argv, "rep");
	//TODO:reads the message trigger type and instantiate the appropriate type of class to sync to. 
		SyncRepublisher<opensimrt_msgs::CommonTimed> sr;
	while(ros::ok())
	{
		sr.run();
		ros::spinOnce();
		sr.r->sleep(); //make sure that every branch has the sleep.
	}

	return 0;
}
