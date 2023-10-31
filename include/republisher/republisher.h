#ifndef REPUBLISHER_HEADER_29052023
#define REPUBLISHER_HEADER_29052023
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include <deque>
#include <optional>
#include "std_msgs/Header.h"

//the message needs to have a header.
template<class T>
class Republisher
{
	public:
		Republisher()
		{
			nh.param<int>("maximum_queue_length", maximum_queue_length, 1000);
			nh.param<int>("input_queue_length", input_queue_length, 10);
			sub = nh.subscribe("input", 10, &Republisher::callback, this);
			pub = nh.advertise<T>("output", input_queue_length);
			ROS_INFO_STREAM("Finished setting up republisher OK.");
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
