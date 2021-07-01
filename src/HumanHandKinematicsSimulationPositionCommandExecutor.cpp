//
// Created by root on 7/1/21.
//

#include "HumanHandKinematicsSimulationPositionCommandExecutor.h"

namespace human {
//=================================================================================================
HumanHandKinematicsSimulationPositionCommandExecutor::HumanHandKinematicsSimulationPositionCommandExecutor(
    dart::dynamics::SkeletonPtr robot,
    const std::string &prefix,
    ::dart::collision::CollisionDetectorPtr collisionDetector,
    ::dart::collision::CollisionGroupPtr collideWith,
    ::dart::collision::CollisionOption collisionOptions)
    : mCollisionDetector(std::move(collisionDetector)),
      mCollideWith(std::move(collideWith)),
      mCollisionOptions(std::move(collisionOptions)),
      mInProgress(false) {
  if (!robot) throw std::invalid_argument("Robot is null!");

  if (mCollisionDetector && mCollideWith) {
    // If a collision group is given and its collision detector does not match
    // mCollisionDetector, set the collision group to an empty collision group.
    if (mCollisionDetector != mCollideWith->getCollisionDetector()) {
      std::cerr << "[HumanHandKinematicSimulationPositionCommandExecutor] "
                << "CollisionDetector of type " << mCollisionDetector->getType()
                << " does not match CollisionGroup's CollisionDetector of type "
                << mCollideWith->getCollisionDetector()->getType() << std::endl;

      ::dart::collision::CollisionGroupPtr newCollideWith
          = mCollisionDetector->createCollisionGroup();
      for (auto i = 0u; i < mCollideWith->getNumShapeFrames(); ++i)
        newCollideWith->addShapeFrame(mCollideWith->getShapeFrame(i));
      mCollideWith = std::move(newCollideWith);
    }
  } else if (mCollisionDetector && !mCollideWith) {
    mCollideWith = mCollisionDetector->createCollisionGroup();
  } else if (!mCollisionDetector && mCollideWith) {
    mCollisionDetector = mCollideWith->getCollisionDetector();
  } else {
    // Default mCollisionDetector to FCL collision detector and mCollideWith to
    // empty collision group.
    mCollisionDetector = dart::collision::FCLCollisionDetector::create();
    mCollideWith = mCollisionDetector->createCollisionGroup();
  }

  //TODO setup executors
}
}

