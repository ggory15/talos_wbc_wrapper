
#include <pal_locomotion_msgs/ActionWithParameters.h>
#include <pal_locomotion_msgs/PushActions.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <property_bag/serialization/property_bag_boost_serialization.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "wbc_action_operation");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<pal_locomotion_msgs::PushActions>(
      "/biped_walking_dcm_controller/push_actions");

  if (!client.waitForExistence(ros::Duration(5.0)))
  {
    ROS_ERROR_STREAM("Cannot connect to the service : " << client.getService());
    return 1;
  }

  pal_locomotion_msgs::PushActions push_actions_request;

  pal_locomotion_msgs::ActionWithParameters new_action;
  new_action.action_type = "pal_locomotion::WBCActions";
  property_bag::PropertyBag parameters;

  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << parameters;
  new_action.action_parameters = ss.str();
  push_actions_request.request.actions.push_back(new_action);

  std::cout << "hi";
  if (client.call(push_actions_request))
  {
    ROS_INFO("Succesfully called service");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }
}
