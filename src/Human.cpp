#include "Human.hpp"

#include <cassert>
#include <mutex>
#include <stdexcept>
#include <string>

#include <aikido/common/RNG.hpp>
#include <aikido/io/yaml.hpp>
#include <aikido/robot/ConcreteManipulator.hpp>
#include <aikido/robot/ConcreteRobot.hpp>
#include <aikido/robot/util.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <dart/common/Timer.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <urdf/model.h>

namespace human {

using dart::dynamics::SkeletonPtr;
using dart::dynamics::BodyNodePtr;
using aikido::constraint::dart::CollisionFreePtr;
using aikido::constraint::TestablePtr;
using aikido::robot::ConcreteManipulatorPtr;
using aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::trajectory::TrajectoryPtr;

const dart::common::Uri humanUrdfUri{
    "package://libhuman/robot/man1.urdf"};
const dart::common::Uri namedConfigurationsUri{
    "TODO"};

namespace {
BodyNodePtr getBodyNodeOrThrow(
    const SkeletonPtr& skeleton, const std::string& bodyNodeName)
{
  auto bodyNode = skeleton->getBodyNode(bodyNodeName);

  if (!bodyNode)
  {
    std::stringstream message;
    message << "Bodynode [" << bodyNodeName << "] does not exist in skeleton.";
    throw std::runtime_error(message.str());
  }

  return bodyNode;
}
} // ns

//==============================================================================
Human::Human(
    aikido::planner::WorldPtr env,
    aikido::common::RNG::result_type rngSeed,
    const dart::common::Uri& humanUrdfUri,
    const dart::common::ResourceRetrieverPtr& retriever)
  : mRng(rngSeed)
  , mWorld(std::move(env))
{
  std::string name = "man1";

  // Load Human.
  mRobotSkeleton = mWorld->getSkeleton(name); // TODO(bhou): set as constant
  if (!mRobotSkeleton)
  {
    dart::utils::DartLoader urdfLoader;
    mRobotSkeleton = urdfLoader.parseSkeleton(humanUrdfUri, retriever);
    mWorld->addSkeleton(mRobotSkeleton);
  }

  if (!mRobotSkeleton)
  {
    throw std::runtime_error("Unable to load Human model.");
  }

  mSpace = std::make_shared<MetaSkeletonStateSpace>(mRobotSkeleton.get());

  // TODO: Enable this.
  configureArm("L", retriever);
  configureArm("R", retriever);

  // TODO!

  // NOTE: Just try and literally load the URDF for now.
  std::cout << "LOADED HUMAN URDF YAY :)" << std::endl;
}

//==============================================================================
std::future<void> Human::executeTrajectory(const TrajectoryPtr& trajectory) const
{
  // TODO!
  throw std::runtime_error("Human -> executeTrajectory() not implemented!");
}

//==============================================================================
boost::optional<Eigen::VectorXd> Human::getNamedConfiguration(
    const std::string& name) const
{
  // TODO!
  throw std::runtime_error("Human -> getNamedConfiguration() not implemented!");
}

//==============================================================================
void Human::setNamedConfigurations(
    std::unordered_map<std::string, const Eigen::VectorXd> namedConfigurations)
{
  // TODO!
  throw std::runtime_error("Human -> setNamedConfiguration() not implemented!");
}

//==============================================================================
std::string Human::getName() const
{
  // TODO!
  throw std::runtime_error("Human -> getName() not implemented!");
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr Human::getMetaSkeleton() const
{
  // TODO!
  throw std::runtime_error("Human -> getMetaSkeleton() not implemented!");
}

//==============================================================================
ConstMetaSkeletonStateSpacePtr Human::getStateSpace() const
{
  // TODO!
  throw std::runtime_error("Human -> getStateSpace() not implemented!");
}

//==============================================================================
void Human::setRoot(Robot* robot)
{
  // TODO!
  throw std::runtime_error("Human -> setRoot() not implemented!");
}

//==============================================================================
void Human::step(const std::chrono::system_clock::time_point& timepoint)
{
  // TODO!
  throw std::runtime_error("Human -> step() not implemented!");
}

//==============================================================================
CollisionFreePtr Human::getSelfCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const
{
  // TODO!
  throw std::runtime_error("Human -> getSelfCollisionConstraint() not implemented!");
}

//==============================================================================
TestablePtr Human::getFullCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const CollisionFreePtr& collisionFree) const
{
  // TODO!
  throw std::runtime_error("Human -> getFullCollisionConstraint() not implemented!");
}

//==============================================================================
std::unique_ptr<aikido::common::RNG> Human::cloneRNG()
{
  return std::move(cloneRNGFrom(mRng)[0]);
}

//==============================================================================
aikido::planner::WorldPtr Human::getWorld()
{
  return mWorld;
}

//==============================================================================
ConcreteManipulatorPtr Human::getRightArm()
{
  // TODO!
  throw std::runtime_error("Human -> getRightArm() not implemented!");
}

//==============================================================================
ConcreteManipulatorPtr Human::getLeftArm()
{
  // TODO!
  throw std::runtime_error("Human -> getLeftArm() not implemented!");
}

//==============================================================================
BodyNodePtr Human::getRightHand()
{
  // TODO!
  throw std::runtime_error("Human -> getRightHand() not implemented!");
}

//==============================================================================
BodyNodePtr Human::getLeftHand()
{
  // TODO!
  throw std::runtime_error("Human -> getLeftHand() not implemented!");
}

//==============================================================================
void Human::configureArm(
    const std::string& armName,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  using dart::dynamics::Chain;

  std::stringstream armStartName;
  armStartName << armName << "Shoulder";

  std::stringstream armEndName;
  armEndName << armName << "Forearm";

  auto armBase = getBodyNodeOrThrow(mRobotSkeleton, armStartName.str());
  auto armEnd = getBodyNodeOrThrow(mRobotSkeleton, armEndName.str());

  auto arm = Chain::create(armBase, armEnd, armName + "_arm");
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(arm.get());

  // Hardcoding to acceleration limits used in OpenRAVE
  // This is necessary because HERB is loaded from URDF, which
  // provides no means of specifying acceleration limits
  arm->setAccelerationLowerLimits(
      Eigen::VectorXd::Constant(arm->getNumDofs(), -2.0));
  arm->setAccelerationUpperLimits(
      Eigen::VectorXd::Constant(arm->getNumDofs(), 2.0));

  // TODO: Set fields, hand.
}

} // ns
