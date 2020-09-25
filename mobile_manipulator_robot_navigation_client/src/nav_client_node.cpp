#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"


void callback_done(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("Robot reached to goal.");
}

void callback_active()
{
    ROS_INFO("Robot start move.");
}

void callback_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    ROS_INFO("Current robot pose is %f %f %f on /map",
                        feedback->base_position.pose.position.x, feedback->base_position.pose.position.y,
                        tf2::getYaw(feedback->base_position.pose.orientation) * 180 / 3.141592654);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_client_node");

    if (argc != 4)
    {
        ROS_INFO("usage: nav_client_node <X> <Y> <THETA(deg)>");
        return 1;
    }


    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    move_base_msgs::MoveBaseGoal goal;
    // goal.goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = atof(argv[1]);
    goal.target_pose.pose.position.y = atof(argv[2]);
    goal.target_pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, atof(argv[3]) * 3.141592654 / 180.0 );

    goal.target_pose.pose.orientation.x = q[0];
    goal.target_pose.pose.orientation.y = q[1];
    goal.target_pose.pose.orientation.z = q[2];
    goal.target_pose.pose.orientation.w = q[3];

    ac.sendGoal(goal, &callback_done, &callback_active, &callback_feedback);

    ros::spin();
    return 0;
}