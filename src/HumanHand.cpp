//
// Created by root on 7/1/21.
//

#include <aikido/control/KinematicSimulationTrajectoryExecutor.hpp>

#include "HumanHand.h"

#undef dtwarn
#define dtwarn (::dart::common::colorErr("Warning", __FILE__, __LINE__, 33))

#undef dtinfo
#define dtinfo (::dart::common::colorMsg("Info", 32))

namespace human {
HumanHand::HumanHand(
    const std::string &name,
    bool simulation,
    dart::dynamics::BodyNodePtr handBaseBodyNode,
    dart::dynamics::BodyNodePtr endEffectorBodyNode,
    std::shared_ptr<dart::collision::BodyNodeCollisionFilter> selfCollisionFilter,
    const ::ros::NodeHandle *node,
    const dart::common::ResourceRetrieverPtr &retriever)
    :
    mName(name),
    mHand(nullptr),
    mSimulation(simulation),
    mHandBaseBodyNode(handBaseBodyNode),
    mEndEffectorBodyNode(endEffectorBodyNode),
    mSelfCollisionFilter(selfCollisionFilter),
    mGrabMetadata(nullptr) {
  if (!mSimulation) {
    if (!node) {
      throw std::runtime_error("ROS node not provided in real.");
    }
    mNode = std::make_unique<::ros::NodeHandle>(*node);
  }

  auto robotSkeleton = mHandBaseBodyNode->getSkeleton();
  std::vector<dart::dynamics::BodyNode *> bodyNodes;
  bodyNodes.reserve(mHandBaseBodyNode->getNumChildBodyNodes());
  for (size_t i = 0; i < mHandBaseBodyNode->getNumChildBodyNodes(); ++i) {
    bodyNodes.emplace_back(mHandBaseBodyNode->getChildBodyNode(i));
  }

  mHand = dart::dynamics::Group::create("Hand", bodyNodes);
  mSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(mHand.get());

  mExecutor = createTrajectoryExecutor(robotSkeleton);
}

//===============================================================================================================
std::shared_ptr<aikido::control::TrajectoryExecutor>
HumanHand::createTrajectoryExecutor(const dart::dynamics::SkeletonPtr &robot) {
  if (mSimulation) {
    return std::make_shared<aikido::control::KinematicSimulationTrajectoryExecutor>(robot);
  } else {
    // TODO Real human?
  }
}

//===================================================================================================
void HumanHand::grab(const dart::dynamics::SkeletonPtr &bodyToGrab) {
  // TODO
}

//==================================================================================================
void HumanHand::ungrab() {
  // TODO
}

//==================================================================================================
std::future<void> HumanHand::executePreshape(const Eigen::Vector2d &preshapeName) {
  // TODO
}

//=================================================================================================
std::future<void> HumanHand::executePreshape(const std::string &preshapeName) {
  // TODO
}

//=================================================================================================
void HumanHand::step(const std::chrono::system_clock::time_point& timepoint) {
  mExecutor->step(timepoint);
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr HumanHand::getMetaSkeleton() const
{
  return mHand;
}

//==============================================================================
dart::dynamics::MetaSkeletonPtr HumanHand::getMetaSkeleton()
{
  return std::const_pointer_cast<dart::dynamics::MetaSkeleton>(
      const_cast<const HumanHand*>(this)->getMetaSkeleton());
}

//==============================================================================
dart::dynamics::BodyNode* HumanHand::getEndEffectorBodyNode() const
{
  return mEndEffectorBodyNode.get();
}

//==============================================================================
dart::dynamics::BodyNode* HumanHand::getHandBaseBodyNode() const
{
  return mHandBaseBodyNode.get();
}

}