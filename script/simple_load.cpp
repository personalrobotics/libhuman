#include <iostream>
#include <Eigen/Dense>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include "Human.hpp"

static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

void waitForUser(const std::string& msg)
{
  std::cout << msg;
  std::cin.get();
}

int main(int argc, char** argv)
{
  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "simple_load");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("simple_load"));

  // Load HERB either in simulation or real based on arguments
  ROS_INFO("Loading Human.");
  human::Human human(env);

  return 0;
}
