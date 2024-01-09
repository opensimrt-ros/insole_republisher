/**
 * @author      : $USER ($USER@29a5851a16ac)
 * @file        : insole_republisher
 * @created     : Tuesday Oct 31, 2023 13:50:57 UTC
 */

#ifndef INSOLE_REPUBLISHER_H

#define INSOLE_REPUBLISHER_H

#include "insole_msgs/InsoleSensorStamped.h"
#include "republisher/republisher.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include <optional>


class InsoleRepublisherWithTimeCorrection: public Republisher<insole_msgs::InsoleSensorStamped>
{
	public:
		InsoleRepublisherWithTimeCorrection()
		{
			nh = ros::NodeHandle("~");
			nh.param<int>("maximum_delay_before_refresh",maximum_delay_before_refresh, 10); //in ms
			int insole_init_time = 0;
			nh.param<int>("insole_time_when_synchronized", insole_init_time,0);
			insole_time_when_synchronized = static_cast<uint32_t>(insole_init_time);
			//now i need to get the ros:time when synchronized from a parameter which is a bit trickier
			int sync_time_secs, sync_time_nsecs;
			nh.param<int>("sync_time_secs", sync_time_secs, 1668695814);
			nh.param<int>("sync_time_nsec", sync_time_nsecs, 0);
			ros_time_when_synchronized = ros::Time{static_cast<uint32_t>(sync_time_secs),static_cast<uint32_t>(sync_time_nsecs)};
			ROS_INFO_STREAM("Ros time when synchronized" << ros_time_when_synchronized);
			ROS_INFO_STREAM("insole_time_when_synchronized" << insole_time_when_synchronized);
			Republisher();

		}
		std::optional<insole_msgs::InsoleSensorStamped> last_gotten_value;
		int maximum_delay_before_refresh = 10;
		ros::Time ros_time_when_synchronized;
		uint32_t insole_time_when_synchronized; 	
		std::optional<uint32_t> first_insole_time_this_measurement;
		ros::Duration this_measurement_time_offset;
		int ros_time_to_insole_time (ros::Time some_time)
		{
			// the times when synchronized should be some class variable i think. 
			//
			auto time_diff = some_time - ros_time_when_synchronized;
			ROS_INFO_STREAM("Time right now - time when I know I was synchronized: " << time_diff);
			int predicted_insole_time = time_diff.toNSec()/1e6 + insole_time_when_synchronized;
			ROS_INFO_STREAM("predicted_insole_time (aka that number that should appear as the time field, but maybe it isn't): "<<predicted_insole_time);
			return predicted_insole_time;
		}
		std::optional<insole_msgs::InsoleSensorStamped> get_latest() 
		{
			//I have a last_gotten_value which is older.
			std::optional<insole_msgs::InsoleSensorStamped> output_value;
			auto el = Republisher::get_latest();
			//this removes the value from the deck. 
			//check if it is time to actually use this value
			//
			//compare it with the clock. if the clock is older/newer idk, my head is a mush
			if (el.has_value())
			{
				auto real_insole_time = el.value().time.data;
				if(!first_insole_time_this_measurement)
				{
					first_insole_time_this_measurement = real_insole_time;
					this_measurement_time_offset = ros::Time::now() - ros_time_when_synchronized;
				}

				//ROS_WARN_STREAM("real_insole_time:"<<real_insole_time);
				int insole_time_difference =ros_time_to_insole_time(ros::Time::now()) - real_insole_time; 
				if(insole_time_difference < 0) //this means that the insole should already have started, right?
				{
					ROS_FATAL_STREAM("insole time difference cannot be negative!!\ninsole_time_difference:" << insole_time_difference);
					//return el;
				}
				bool choose_older_value_condition = insole_time_difference < maximum_delay_before_refresh;
				if(choose_older_value_condition)
				{
					ROS_INFO_STREAM("chosen older value");
					//then i need to put back the get latest
					input_queue.push_front(el.value());
					output_value = last_gotten_value;
				}
				else
				{
					output_value = el;
					//but we also need to correct the time, right?
					output_value.value().header.stamp = convert_insole_to_ros_time(real_insole_time); 
				}

				last_gotten_value = output_value; //because of reasons.
			}
			return output_value;
		}
		ros::Time convert_insole_to_ros_time(int time_insole)
		{
			int time_diff = time_insole - insole_time_when_synchronized;
			if (time_diff<0)
			{

				ROS_FATAL_STREAM("current insole time is smaller than when insole was synchronized?!?!");
			}
			ROS_INFO_STREAM("time_diff:" << time_diff);
			auto offset_time = ros::Duration().fromSec(double(time_diff)/1000); 
			ROS_INFO_STREAM("duration to add to time" <<offset_time);
			auto converted_time =ros_time_when_synchronized+offset_time; 
			ROS_WARN_STREAM("converted time:" <<converted_time);
			return converted_time; 

		}

};


#endif /* end of include guard INSOLE_REPUBLISHER_H */

