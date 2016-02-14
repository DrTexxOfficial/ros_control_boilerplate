#include <ros/ros.h>
#include <ros_control_boilerplate/read_joint_states.h>
#include <ros_control_boilerplate/write_joint_cmd.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "read_joint_states_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ros_control_boilerplate::read_joint_states>("read_joint_states");
  ros_control_boilerplate::read_joint_states srv;
  srv.request.input = float(1234); //sending a random input value as request
  if (client.call(srv))
  {
    ROS_INFO("Joint0: %f", (float)srv.response.joint0);
    ROS_INFO("Joint1: %f", (float)srv.response.joint1);
    ROS_INFO("Joint2: %f", (float)srv.response.joint2);
    ROS_INFO("Joint3: %f", (float)srv.response.joint3);
    ROS_INFO("Joint4: %f", (float)srv.response.joint4);

  }
  else
  {
    ROS_ERROR("Failed to call service read_joint_states");
    return 1;
  }

  return 0;
}
