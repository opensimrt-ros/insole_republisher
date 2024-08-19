#ifndef REPUBLISHER_HEADER_29052023
#define REPUBLISHER_HEADER_29052023
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "republisher/delayConfig.h"
#include "republisher/filter.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Header.h"
#include "tf/transform_broadcaster.h"

// this is no longer a republisher this was harder than i thought. 
//the message needs to have a header.
template<class T>
class Republisher
{
	public:
		ros::Rate* r;
		double* old_time;
		ros::Duration delay;
		tf::TransformBroadcaster tf;
		bool debug_publish_zero_cop, debug_publish_fixed_force;
		ros::Publisher wrench_publisher_f, wrench_publisher, imu_publisher;
		
		std::string foot_center_name;

		RosOpenSimRTFilter* wfilter;
		AppropriateTime<T> at;
		Republisher()
		{
			ROS_INFO_STREAM("this started");
			wfilter = new RosOpenSimRTFilter(nh,6);
			nh.getParam("foot_center_name", foot_center_name);
			
			nh.param<int>("input_queue_length", input_queue_length, 10);
			sub = nh.subscribe("input", 10, &Republisher::callback, this);
			pub = nh.advertise<T>("output", input_queue_length);
			double delay_seconds;
			nh.param<double>("wrench_delay", delay_seconds, 0); 
			delay.fromSec(delay_seconds);
			//if (delay_seconds != 0)
			//	ROS_WARN("Delay is currently unused. your header will be the same as the initial header for all messages of this node!!!");

			nh.param<bool>("debug_publish_zero_cop", debug_publish_zero_cop, false);
			nh.param<bool>("debug_publish_fixed_force", debug_publish_fixed_force, false);
			
			ros::NodeHandle n; //publish on global namespace:w

			wrench_publisher = n.advertise<geometry_msgs::WrenchStamped>("wrench",1);
			wrench_publisher_f = n.advertise<geometry_msgs::WrenchStamped>("wrench_filtered",1);
			imu_publisher = n.advertise<sensor_msgs::Imu>("imu_raw",1);

			ROS_INFO_STREAM("Finished setting up republisher OK.");
		}

		void publish_imu(T msg_insole)
		{
			auto i_msg = sensor_msgs::Imu();
			i_msg = msg_insole.imu;
			i_msg.header= msg_insole.header;
			i_msg.header.frame_id = foot_center_name;
			imu_publisher.publish(i_msg);	
		}

		geometry_msgs::WrenchStamped publish_wrench(T msg_insole)
		{
			auto w = geometry_msgs::WrenchStamped();
			w.header = msg_insole.header;
			//However the wrench if applied to the COP not to the frame of the insole
			w.header.frame_id = msg_insole.ts.child_frame_id;
			w.wrench = msg_insole.wrench;
			if (debug_publish_fixed_force)
			{
				w.wrench.force = geometry_msgs::Vector3();
				w.wrench.force.y = 100;
			}
			wrench_publisher.publish(w);
			return w;

		}
		geometry_msgs::TransformStamped publish_tf(T msg_insole)
		{
			auto st = geometry_msgs::TransformStamped();
			st.header = msg_insole.header;
		
			st.child_frame_id = msg_insole.ts.child_frame_id;
			st.transform = msg_insole.ts.transform;

			if (st.header.frame_id == "")
			{
				ROS_ERROR("tf has no source frame_id");
			}
			if (st.child_frame_id == "")
			{
				ROS_ERROR("tf has no child_frame_id");
			}
			if (debug_publish_zero_cop)
			{
				st.transform.translation = geometry_msgs::Vector3();
			}

			tf.sendTransform(st);
			return st;
		}
		void reconfigure_delay_callback(republisher::delayConfig &config, uint32_t level){
			ROS_INFO("Setting delay to new value:%f[s]", config.side_delay);

			delay.fromSec(config.side_delay);
			at.set_delay(delay);
		}

		void callback(T msg_insole)
		{
			ROS_DEBUG_STREAM("received value!");

			if (!at.initial_time)
			{
				at.set_initial_time(msg_insole.header,delay);
				*old_time = at.now(msg_insole.header);
			}

			double time = at.now(msg_insole.header);
			msg_insole = at.shift(msg_insole);
			if (time<=*old_time)
			{
				ROS_ERROR_STREAM("time is smaller than previous time. this is a problem: time:" << time << "*old_time" << old_time);
				return;
			}
			ROS_DEBUG_STREAM(time);
			///////wrench_publisher.publish(w);
			auto w = publish_wrench(msg_insole);
			auto st = publish_tf(msg_insole);
			publish_imu(msg_insole);
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
				st.child_frame_id = msg_insole.ts.child_frame_id + "_filtered";
				if (x_filtered.isValid)
				{
					ROS_DEBUG_STREAM("Filter is valid");
					w.header.frame_id = st.child_frame_id; //this is the correct frame for filtered, right?
					w.wrench.force.x = x_filtered.x[0];
					w.wrench.force.y = x_filtered.x[1];
					w.wrench.force.z = x_filtered.x[2];
					ROS_DEBUG_STREAM(x <<"  "<<x_filtered.x);
					st.transform.translation.x = x_filtered.x[3];
					st.transform.translation.y = x_filtered.x[4];
					st.transform.translation.z = x_filtered.x[5];
					///////////tf.sendTransform(st);
					// check if the tf trasnformaitons are complete
					tf.sendTransform(st);
					//we will only publish if everything is okay with the transform too. 
					wrench_publisher_f.publish(w);
				}
			}
			*old_time = time;
		}


		ros::NodeHandle nh{"~"};
	private:
		ros::Subscriber sub;
		ros::Publisher pub;
		int input_queue_length;
		std::optional<T> last_value;
};

#endif
