#include "ros/ros.h"
#include "std_msgs/String.h"

#include "trajectory_generator_node/GetTrajectory.h"

#include <thread>
#include <string>
#include <mutex>
#include <vector>

ros::NodeHandle* node;

ros::ServiceClient get_trajectory_service;

ros::ServiceClient& get_trajectory_service_get()
{
	if (node && !get_trajectory_service)
	{
		get_trajectory_service = node->serviceClient<trajectory_generator_node::GetTrajectory>("get_trajectory", true);
	}
	return get_trajectory_service;
};

void get_trajectory(std::string trajectory_name)
{
	if (get_trajectory_service_get())
	{
		trajectory_generator_node::GetTrajectory gt;
		gt.request.path_name = trajectory_name;
		bool service_success = get_trajectory_service_get().call(gt);
		if (service_success)
		{
			//gt.response.
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_follower_node");

	ros::NodeHandle n;

	node = &n;

	ros::spin();
	return 0;
}