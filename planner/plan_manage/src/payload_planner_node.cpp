#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/replan_fsm.h>
#include "plan_manage/backward.hpp"

// for better debugging
namespace backward {
backward::SignalHandling sh;
}

using namespace payload_planner;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "payload_planner_node");
  ros::NodeHandle nh("~");

  ReplanFSM payload_replan;
  payload_replan.init(nh);

  ros::spin();

  return 0;
}
