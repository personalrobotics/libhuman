//
// Created by root on 7/1/21.
//

#ifndef LIBHUMAN_INCLUDE_HUMANHANDKINEMATICSSIMULATIONPOSITIONCOMMANDEXECUTOR_H_
#define LIBHUMAN_INCLUDE_HUMANHANDKINEMATICSSIMULATIONPOSITIONCOMMANDEXECUTOR_H_

#include <aikido/control/PositionCommandExecutor.hpp>
#include <dart/dart.hpp>

namespace human {
class HumanHandKinematicsSimulationPositionCommandExecutor : public aikido::control::PositionCommandExecutor {
  /// Constructor.
  ///
  /// \param robot Robot to construct executor for
  /// \param prefix String (either "/right/" or "/left/") to specify hand
  /// \param collisionDetector CollisionDetector to check finger collisions if nullptr,
  ///     default to FCLCollisionDetector
  /// \param collideWith CollisionGroup to check finger collisions. If nullptr, default to empty CollisionGroup.
  /// \param collisionOptions
  ///     Default is (enableContact=false, binaryCheck=true, maxNumContacts=1.) See dart/collision/Option.h for more
  ///     information
  HumanHandKinematicsSimulationPositionCommandExecutor(
      dart::dynamics::SkeletonPtr robot,
      const std::string &prefix,
      ::dart::collision::CollisionDetectorPtr collisionDetector = nullptr,
      ::dart::collision::CollisionGroupPtr collideWith = nullptr,
      ::dart::collision::CollisionOption collisionOptions
      = ::dart::collision::CollisionOption(false, 1));

 private:
  ::dart::collision::CollisionDetectorPtr mCollisionDetector;
  ::dart::collision::CollisionGroupPtr mCollideWith;
  ::dart::collision::CollisionOption mCollisionOptions;
  bool mInProgress;
};
}

#endif //LIBHUMAN_INCLUDE_HUMANHANDKINEMATICSSIMULATIONPOSITIONCOMMANDEXECUTOR_H_
