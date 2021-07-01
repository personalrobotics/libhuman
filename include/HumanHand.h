//
// Created by root on 7/1/21.
//

#ifndef LIBHUMAN_INCLUDE_HUMANHAND_H_
#define LIBHUMAN_INCLUDE_HUMANHAND_H_

#include <aikido/common/pointers.hpp>
#include <aikido/robot/Robot.hpp>
#include <aikido/robot/Hand.hpp>
#include <dart/dart.hpp>
#include <ros/ros.h>

#include "HumanHandKinematicsSimulationPositionCommandExecutor.h"

namespace human {

AIKIDO_DECLARE_POINTERS(HumanHand)

class HumanHand : public aikido::robot::Hand {
 public:
  /// Creates an instance of a HumanHand.
  ///
  /// \param name Name of the hand, either "left" or "right".
  /// \param simulation True if running in simulation mode.
  /// \param handBaseBodyNode Body node which is the root of all fingers.
  /// \param endEffectorBodyNode End-effector boday node. Must be the link that represents the palm of an humanHand,
  ///        for which inverse kinematics is solved.
  /// \param selfCollisionFilter CollisionFilter used for self-collision checking for the whole human.
  /// \param node ROS node. Required for running in real. May be nullptr if simulation is true.
  /// \param retriever Resource retriever for retrieving preshapes and end-effector transforms
  HumanHand(const std::string &name,
            bool simulation,
            dart::dynamics::BodyNodePtr handBaseBodyNode,
            dart::dynamics::BodyNodePtr endEffectorBodyNode,
            std::shared_ptr<dart::collision::BodyNodeCollisionFilter> selfCollisionFilter,
            const ::ros::NodeHandle *node,
            const dart::common::ResourceRetrieverPtr &retriever);

  virtual ~HumanHand() = default;

 public:
  void grab(const dart::dynamics::SkeletonPtr &bodyToGrab) override;

  void ungrab() override;

  std::future<void> executePreshape(const std::string &preshapeName) override;

  std::future<void> executePreshape(const Eigen::Vector2d &preshape);

  void step(const std::chrono::system_clock::time_point &timepoint) override;

  // Documentation inherited.
  dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const override;

  // Documentation inherited.
  dart::dynamics::MetaSkeletonPtr getMetaSkeleton() override;

  // Documentation inherited.
  dart::dynamics::BodyNode *getEndEffectorBodyNode() const override;

  // Documentation inherited.
  dart::dynamics::BodyNode *getHandBaseBodyNode() const override;

 private:
  /// Create controllers in real or simulation.
  ///
  /// \param robot Robot to construct executor for
  std::shared_ptr<aikido::control::TrajectoryExecutor> createTrajectoryExecutor(
      const dart::dynamics::SkeletonPtr &robot);

  /// Name of the hand
  const std::string mName;

  /// Hand metaskeleton consisting of the nodes rooted at mHandBaseBodyNode.
  dart::dynamics::GroupPtr mHand;

  /// Statespace for the hand.
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mSpace;

  /// Whther hand is runnign in simulation mode
  const bool mSimulation;

  /// Body node which is the root link for all fingers
  dart::dynamics::BodyNodePtr mHandBaseBodyNode;

  /// End-effector body node, for which IK is created
  dart::dynamics::BodyNodePtr mEndEffectorBodyNode;

  /// CollisionFilter used for self-collision checking for the whole human
  std::shared_ptr<dart::collision::BodyNodeCollisionFilter> mSelfCollisionFilter;

  /// Metadata about the object currently being grabbed
  /// TODO: change this to grab multiple objects
  std::unique_ptr<aikido::robot::GrabMetadata> mGrabMetadata;

  /// ROS node. Required for running in real.
  std::unique_ptr<::ros::NodeHandle> mNode;

  /// Hand position command executor (may be real if simulation is False)
  std::shared_ptr<aikido::control::TrajectoryExecutor> mExecutor;
};

}

#endif //LIBHUMAN_INCLUDE_HUMANHAND_H_
