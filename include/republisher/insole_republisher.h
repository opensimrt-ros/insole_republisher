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
#include "ros/message_traits.h"
#include "ros/time.h"
#include <optional>


class InsoleRepublisherWithTimeCorrection: public Republisher<insole_msgs::InsoleSensorStamped>
{
	InsoleRepublisherWithTimeCorrection()
	{
		nh.param<int>("maximum_delay_before_refresh",maximum_delay_before_refresh, 10); //in ms
		int insole_init_time = 0;
		nh.param<int>("insole_time_when_synchronized", insole_init_time,0);
		insole_time_when_synchronized = static_cast<uint32_t>(insole_init_time);
		//now i need to get the ros:time when synchronized from a parameter which is a bit trickier
		int sync_time_secs, sync_time_nsecs;
		nh.param<int>("sync_time_secs", sync_time_secs, 1668695814);
		nh.param<int>("sync_time_nsec", sync_time_nsecs, 0);
		ros_time_when_synchronized = ros::Time{static_cast<uint32_t>(sync_time_secs),static_cast<uint32_t>(sync_time_nsecs)};
		Republisher();

	}
	std::optional<insole_msgs::InsoleSensorStamped> last_gotten_value;
	int maximum_delay_before_refresh = 10;
	ros::Time ros_time_when_synchronized;
	uint32_t insole_time_when_synchronized; 	
	int ros_time_to_insole_time (ros::Time some_time)
	{
		// the times when synchronized should be some class variable i think. 
		//
		auto time_diff = some_time - ros_time_when_synchronized;
		return time_diff.toNSec()/1e6 + insole_time_when_synchronized;
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
		bool choose_older_value_condition = ros_time_to_insole_time(ros::Time::now()) - el.value().time.data > maximum_delay_before_refresh;
		if(choose_older_value_condition)
		{
			//then i need to put back the get latest
			input_queue.push_front(el.value());
			output_value = last_gotten_value;
		}
		else
		{
			output_value = el;
		}

		last_gotten_value = output_value; //because of reasons.
		return el;
	}


};


#endif /* end of include guard INSOLE_REPUBLISHER_H */

