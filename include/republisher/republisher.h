#ifndef REPUBLISHER_HEADER_29052023
#define REPUBLISHER_HEADER_29052023
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include <deque>
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
		void publish_one()
		{
			if (input_queue.size()>0)
			{
				T el =input_queue.front();
				std_msgs::Header h;
				h.stamp = ros::Time::now();
				el.header = h;
				pub.publish(el);
				input_queue.pop_front();
			}
			else
			{
				pub.publish(last_value);
				//ROS_WARN("input_queue is empty");
			}

		}
	private:
		ros::Subscriber sub;
		ros::Publisher pub;
		ros::NodeHandle nh{"~"};
		int input_queue_length, maximum_queue_length;
		std::deque<T> input_queue;
		T last_value;
		void callback(T msg_insole)
		{
			last_value = msg_insole;
			input_queue.push_back(msg_insole);
			while (input_queue.size()>maximum_queue_length)
			{
				ROS_WARN_THROTTLE(20, "maximum_queue_length exceeded dropping old items");
				input_queue.pop_back();

			}
		}
};

#endif
