#ifndef REPUBLISHER_HEADER_29052023
#define REPUBLISHER_HEADER_29052023
#include "geometry_msgs/WrenchStamped.h"
#include "insole_msgs/InsoleSensorStamped.h"
#include "republisher/filter.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include <deque>
#include <optional>
#include "std_msgs/Header.h"
#include "tf/transform_broadcaster.h"

struct PublicationBuffer
{
	geometry_msgs::WrenchStamped w_oversampled;
	geometry_msgs::WrenchStamped w_filtered;
	geometry_msgs::TransformStamped t_oversampled;
	geometry_msgs::TransformStamped t_filtered;
};

//the message needs to have a header.
template<class T>
class Republisher
{
	public:
		ros::Rate* r;
		double old_time;
		ros::Duration delay;
		tf::TransformBroadcaster tf;
		bool debug_publish_zero_cop, debug_publish_fixed_force;
		ros::Publisher wrench_publisher = nh.advertise<geometry_msgs::WrenchStamped>("wrench_oversampled",1);
		ros::Publisher wrench_publisher_f = nh.advertise<geometry_msgs::WrenchStamped>("wrench_filtered",1);

		RosOpenSimRTFilter* wfilter = new RosOpenSimRTFilter(nh,6);
		AppropriateTime at;
		PublicationBuffer pb;
		Republisher()
		{
			nh.param<int>("maximum_queue_length", maximum_queue_length, 1000);
			nh.param<int>("input_queue_length", input_queue_length, 10);
			sub = nh.subscribe("input", 10, &Republisher::callback, this);
			pub = nh.advertise<T>("output", input_queue_length);
			double rate;
			nh.param<double>("rate", rate, 100);
			r = new ros::Rate(rate);
			double delay_seconds;
			nh.param<double>("wrench_delay", delay_seconds, 0); 
			delay.fromSec(delay_seconds);

			nh.param<bool>("debug_publish_zero_cop", debug_publish_zero_cop, false);
			nh.param<bool>("debug_publish_fixed_force", debug_publish_fixed_force, false);

			ROS_INFO_STREAM("Finished setting up republisher OK.");
		}

		void run()
		{
			std::optional<insole_msgs::InsoleSensorStamped> el_o = get_latest();
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
				}
			}
			old_time = time;
		}

		std::optional<T> get_latest()
		{
			if (input_queue.size()>0)
			{
				T el =input_queue.front();
				std_msgs::Header h;
				h.stamp = ros::Time::now(); // listen here buddy, this is incorrecto. I am removing any delay I had from the initial message!
				h.frame_id = el.header.frame_id;
				el.header = h;
				input_queue.pop_front();
				ROS_DEBUG_STREAM(el);
				return std::optional<T>(el);
			}
			else
			{
				if (!last_value)
				{//ROS_ERROR_STREAM("variable not initialized!!!");
					return std::nullopt;
				}
				//we restamp the last value so it is new!
				last_value.value().header.stamp = ros::Time::now();
				//ROS_WARN("input_queue is empty");
				return last_value;
			}

		}

		void publish_one()
		{
			pub.publish(get_latest());
		}
		ros::NodeHandle nh{"~"};
		std::deque<T> input_queue;
	private:
		ros::Subscriber sub;
		ros::Publisher pub;
		int input_queue_length, maximum_queue_length;
		std::optional<T> last_value;
		void callback(T msg_insole)
		{
			ROS_DEBUG_STREAM("received value!");
			last_value = std::optional<T>(msg_insole);
			input_queue.push_back(msg_insole);
			while (input_queue.size()>maximum_queue_length)
			{
				ROS_WARN_THROTTLE(20, "maximum_queue_length exceeded dropping old items");
				input_queue.pop_back();

			}
		}
};

#endif
