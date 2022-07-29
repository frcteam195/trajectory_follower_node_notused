#include "ros/ros.h"
#include "std_msgs/String.h"

#include "trajectory_follower_node/StartTrajectory.h"
#include "trajectory_follower_node/FollowerStatus.h"
#include "trajectory_generator_node/GetTrajectory.h"
#include "trajectory_generator_node/OutputTrajectory.h"

#include "ck_utilities/Logger.hpp"
#include "ck_utilities/planners/DriveMotionPlanner.hpp"

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

ros::NodeHandle *node;

ros::ServiceClient get_trajectory_service;

std::recursive_mutex running_traj_lock;
trajectory_generator_node::OutputTrajectory running_trajectory;
std::string running_trajectory_name;
std::atomic_bool is_running_traj{false};

trajectory_follower_node::FollowerStatus follower_status;

ck::planners::DriveMotionPlanner motion_planner;

ros::ServiceClient &get_trajectory_service_get()
{
    if (node && !get_trajectory_service)
    {
        get_trajectory_service = node->serviceClient<trajectory_generator_node::GetTrajectory>("get_trajectory", true);
    }
    return get_trajectory_service;
};

bool get_trajectory(std::string trajectory_name, trajectory_generator_node::OutputTrajectory &traj)
{
    if (get_trajectory_service_get())
    {
        trajectory_generator_node::GetTrajectory gt;
        gt.request.path_name = trajectory_name;
        bool service_success = get_trajectory_service_get().call(gt);
        if (service_success)
        {
            traj = gt.response.trajectory;
        }
        return service_success;
    }
    return false;
}

void zero_follower_status(void)
{
    follower_status.leftMotorAccelRadPerSec2 = 0.0;
    follower_status.leftMotorFeedforwardVoltage = 0.0;
    follower_status.leftMotorOutputRadPerSec = 0.0;

    follower_status.rightMotorAccelRadPerSec2 = 0.0;
    follower_status.rightMotorFeedforwardVoltage = 0.0;
    follower_status.rightMotorOutputRadPerSec = 0.0;
}

bool start_trajectory(trajectory_follower_node::StartTrajectory::Request &request, trajectory_follower_node::StartTrajectory::Response &response)
{
    if (is_running_traj)
    {
        std::lock_guard<std::recursive_mutex> lock(running_traj_lock);
        ck::log_error << "Failed to start " << request.traj_name << "! Already running trajectory" << running_trajectory_name << "!" << std::flush;
    }
    else
    {
        trajectory_generator_node::OutputTrajectory t;
        bool success = get_trajectory(request.traj_name, t);
        if (success)
        {
            ck::log_info << "Starting trajectory: " << request.traj_name << std::flush;
            std::lock_guard<std::recursive_mutex> lock(running_traj_lock);
            running_trajectory_name = request.traj_name;
            running_trajectory = t;
            is_running_traj = true;
            return true;
        }
        else
        {
            ck::log_error << "Failed to start " << request.traj_name << "! Could not get trajectory!" << std::flush;
        }
    }

    (void)response;
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_follower_node");

    ros::NodeHandle n;

    node = &n;
    ros::ServiceServer start_traj_service = node->advertiseService("start_trajectory", start_trajectory);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        if (is_running_traj)
        {
            std::lock_guard<std::recursive_mutex> lock(running_traj_lock);
            // TODO:...
        }

        /*
                    if (!mOutputData.trajectoryActive) {
                        if (mInputData.beginTrajectory) {
                            mCurrentTrajectory_id = mInputData.trajectoryID;
                            motionPlanner.reset();

                            System.out.println("Getting trajectory from map! mCurrentTrajectory_id = " + mCurrentTrajectory_id);
                            mTrajectory = new TrajectoryIterator<>(new TimedView<>(TrajectoryGenerator.getInstance()
                                    .trajectoryLookupMap.get(mCurrentTrajectory_id)));
                            motionPlanner.setTrajectory(mTrajectory);
                            mOutputData.trajectoryActive = true;
                            mOutputData.trajectoryCompleted = false;
                        }
                    }

        if (mOutputData.trajectoryActive && !mOutputData.trajectoryCompleted)
        {
            DriveOutput d = motionPlanner.update(mInputData.timestamp, mInputData.poseInches);
            mOutputData.leftMotorOutputRadPerSec = d.left_velocity;
            mOutputData.leftMotorFeedforwardVoltage = d.left_feedforward_voltage;
            mOutputData.leftMotorAccelRadPerSec2 = d.left_accel;

            mOutputData.rightMotorOutputRadPerSec = d.right_velocity;
            mOutputData.rightMotorFeedforwardVoltage = d.right_feedforward_voltage;
            mOutputData.rightMotorAccelRadPerSec2 = d.right_accel;
        }

        */

        if (motion_planner.isDone())
        {
            zero_follower_status();
            follower_status.trajectoryCompleted = true;
            follower_status.trajectoryActive = false;
        }

        if (true /* mInputData.forceStop*/)
        {
            zero_follower_status();
            motion_planner.reset();
            follower_status.trajectoryCompleted = true;
            follower_status.trajectoryActive = false;
        }

        static ros::Publisher trajectory_follower_publisher = node->advertise<trajectory_follower_node::FollowerStatus>("/TrajcetoryFollowerStatus", 1);
        trajectory_follower_publisher.publish(follower_status);

        rate.sleep();
    }

    return 0;
}
