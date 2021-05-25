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

using dart::dynamics::SkeletonPtr;

void waitForUser(const std::string& msg)
{
  std::cout << msg;
  std::cin.get();
}

const SkeletonPtr makeBodyFromURDF(
    const std::shared_ptr<aikido::io::CatkinResourceRetriever>
        resourceRetriever,
    const std::string& uri,
    const Eigen::Isometry3d& transform)
{
  dart::utils::DartLoader urdfLoader;
  const SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

  if (!skeleton)
    throw std::runtime_error("unable to load '" + uri + "'");

  dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
      ->setTransform(transform);
  return skeleton;
}

int main(int argc, char** argv)
{
  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "simple_load");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("simple_load"));

  // Load the human.
  ROS_INFO("Loading Human.");
  human::Human human(env);

  // Start Visualization Topic
  static const std::string topicName = topicName + "/simple_load";

  // Start the RViz viewer.
  ROS_INFO_STREAM(
      "Starting viewer. Please subscribe to the '"
      << topicName
      << "' InteractiveMarker topic in RViz.");
  aikido::rviz::InteractiveMarkerViewer viewer(topicName, baseFrameName, env);

  // Add Human to the viewer.
  viewer.setAutoUpdate(true);

  // Add a loaded table to the scene.
  const std::string tableURDFUri(
      "package://pr_assets/data/furniture/uw_demo_table.urdf");

  Eigen::Isometry3d tablePose = Eigen::Isometry3d::Identity();
  tablePose.translation() = Eigen::Vector3d(1.0, 0.0, 0);
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  tablePose.linear() = rot;

  // Load table
  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();
  SkeletonPtr table = makeBodyFromURDF(resourceRetriever, tableURDFUri, tablePose);

  // Add all objects to World
  human.getWorld()->addSkeleton(table);

  waitForUser("Press [ENTER] to exit: ");

  return 0;
}
