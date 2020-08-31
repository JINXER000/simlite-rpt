
#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_set_states_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    gazebo_msgs::SetModelState objstate;

    objstate.request.model_state.model_name = "unit_cylinder_0";
    objstate.request.model_state.pose.position.x = 0.5;
    objstate.request.model_state.pose.position.y = 0.5;
    objstate.request.model_state.pose.position.z = 0.5;
    objstate.request.model_state.pose.orientation.w = 1;
    objstate.request.model_state.pose.orientation.x = 0;
    objstate.request.model_state.pose.orientation.y = 0;
    objstate.request.model_state.pose.orientation.z = 0;
    objstate.request.model_state.twist.linear.x = 0.0;
    objstate.request.model_state.twist.linear.y = 0.0;
    objstate.request.model_state.twist.linear.z = 0.0;
    objstate.request.model_state.twist.angular.x = 0.0;
    objstate.request.model_state.twist.angular.y = 0.0;
    objstate.request.model_state.twist.angular.z = 0.0;
    objstate.request.model_state.reference_frame = "world";

    client.call(objstate);
}
