#ifndef REPUBLISHER_HEADER_29052023
#define REPUBLISHER_HEADER_29052023
#include "geometry_msgs/WrenchStamped.h"
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
	geometry_msgs::WrenchStamped w;
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
		ros::Publisher wrench_publisher_f;

		RosOpenSimRTFilter* wfilter;
		AppropriateTime at;
		PublicationBuffer pb;
		Republisher()
		{
			ROS_INFO_STREAM("this started");
			wfilter = new RosOpenSimRTFilter(nh,6);
			nh.param<int>("maximum_queue_length", maximum_queue_length, 1000);
			nh.param<int>("input_queue_length", input_queue_length, 10);
			sub = nh.subscribe("input", 10, &Republisher::callback, this);
			pub = nh.advertise<T>("output", input_queue_length);
			double delay_seconds;
			nh.param<double>("wrench_delay", delay_seconds, 0); 
			delay.fromSec(delay_seconds);

			nh.param<bool>("debug_publish_zero_cop", debug_publish_zero_cop, false);
			nh.param<bool>("debug_publish_fixed_force", debug_publish_fixed_force, false);
		
			wrench_publisher_f = nh.advertise<geometry_msgs::WrenchStamped>("wrench_filtered",1);

			ROS_INFO_STREAM("Finished setting up republisher OK.");
		}

		void run()
		{
			std::optional<T> el_o = get_latest();
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
			///////wrench_publisher.publish(w);
			auto st = geometry_msgs::TransformStamped();
			st.header = el.header;
			st.header.frame_id = el.ts.header.frame_id;
			st.transform = el.ts.transform;
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
				st.child_frame_id = el.ts.child_frame_id + "_filtered";
				if (x_filtered.isValid)
				{
					ROS_DEBUG_STREAM("Filter is valid");
					w.header.frame_id = st.child_frame_id; //this is the correct frame for filtered, right?
					if (debug_publish_fixed_force)
					{
						w.wrench.force = geometry_msgs::Vector3();
						w.wrench.force.y = 100;
					}
					else
					{
						w.wrench.force.x = x_filtered.x[0];
						w.wrench.force.y = x_filtered.x[1];
						w.wrench.force.z = x_filtered.x[2];
					}
					ROS_DEBUG_STREAM(x <<"  "<<x_filtered.x);
					///////wrench_publisher_f.publish(w);
					if (debug_publish_zero_cop)
					{
						st.transform.translation = geometry_msgs::Vector3();
					}
					else
					{
						st.transform.translation.x = x_filtered.x[3];
						st.transform.translation.y = x_filtered.x[4];
						st.transform.translation.z = x_filtered.x[5];
					}
					///////////tf.sendTransform(st);
					pb.t_filtered = st;
					pb.w = w;
					// check if the tf trasnformaitons are complete
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
							wrench_publisher_f.publish(pb.w);
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

		virtual std::optional<T> get_latest()
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
