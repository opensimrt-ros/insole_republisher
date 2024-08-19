/**
 * @author      : $USER ($USER@29a5851a16ac)
 * @file        : filter
 * @created     : Tuesday Oct 31, 2023 15:22:49 UTC
 */

#ifndef FILTER_H

#define FILTER_H
#include "SignalProcessing.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "std_msgs/Header.h"

class RosOpenSimRTFilter
{
	public:
		OpenSimRT::LowPassSmoothFilter::Parameters filterParam;
		bool publish_filtered;	
		double cutoffFreq;
		int splineOrder, memory, delay;
		OpenSimRT::LowPassSmoothFilter * ofilter;
		double old_time = -10000;
		RosOpenSimRTFilter(ros::NodeHandle nh, int numSignals)
		{
			nh.param<bool>("filter_output",publish_filtered, true);
			nh.param<double>("cutoff_freq", cutoffFreq, 0.0);
			nh.param<int>("memory", memory, 0);
			nh.param<int>("spline_order", splineOrder, 0);
			nh.param<int>("delay", delay, 0);

			filterParam.numSignals = numSignals; 
			filterParam.memory = memory;
			filterParam.delay = delay;
			filterParam.cutoffFrequency = cutoffFreq;
			filterParam.splineOrder = splineOrder;
			filterParam.calculateDerivatives = true;
			ofilter = new OpenSimRT::LowPassSmoothFilter(filterParam);
		}
		OpenSimRT::LowPassSmoothFilter::Output filter(double t, SimTK::Vector v)
		{
			if (t <= old_time)
			{
				ROS_FATAL_STREAM("time added to filter: "<< t <<  " is smaller than previous_time: "<< old_time<<". how can this be?");
				return ofilter->filter({old_time,v});
			}
			old_time = t;
			return ofilter->filter({t,v});
		}
};

template<class T>
class AppropriateTime
{
	public:
		std::optional<ros::Time> initial_time;

		void set_initial_time(const std_msgs::Header h, const ros::Duration d)
		{
			ROS_WARN_STREAM("using this header:"<<h << " and this duration"<< d);
			initial_h = h;
			if (initial_h.value().stamp.toSec()<5)
				ROS_FATAL_STREAM("You cannot have such small stamped values or you won't be able to go back in time (Unix time is always positive!)");

			set_delay(d);
		}
		double now(const std_msgs::Header h)
		{
			try {
				ros::Duration d = h.stamp - initial_time.value();
				return d.toSec();
			}
			catch(std::runtime_error& ex) {
				ROS_ERROR("Exception: [%s]", ex.what());
				return h.stamp.toSec();
			}
		}

		T shift(T msg)
		{
			if (delay.has_value()){
				try {
					msg.header.stamp += delay.value();
				}
				catch(std::runtime_error& ex) {
					ROS_ERROR("Exception: [%s]", ex.what());
				}}
			else
				ROS_ERROR("delay not set!");
			return msg;
		}
		void set_delay(const ros::Duration d)
		{
			if(initial_h.has_value())
			{
				delay = d;

				ROS_WARN_STREAM("Delay set to "<< delay.value());
				try {
					initial_time = initial_h.value().stamp + d;
				}
				catch(std::runtime_error& ex) {
					ROS_ERROR("Exception: [%s]", ex.what());
				}
			}
			else
				ROS_WARN_STREAM("I cannot set own time yet. First message not received.");
		}
	private:
		std::optional<std_msgs::Header> initial_h;
		std::optional<ros::Duration> delay;
};



#endif /* end of include guard FILTER_H */

