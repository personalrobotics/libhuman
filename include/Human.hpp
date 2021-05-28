#ifndef LIBHUMAN_HUMAN_HPP_
#define LIBHUMAN_HUMAN_HPP_

#include <future>
#include <memory>
#include <Eigen/Core>
#include <aikido/common/RNG.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/robot/ConcreteManipulator.hpp>
#include <aikido/robot/ConcreteRobot.hpp>
#include <aikido/robot/util.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <boost/optional.hpp>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/dart.hpp>

namespace human {

/// URI to retrieve Human URDF from.
extern const dart::common::Uri humanUrdfUri;

/// URI to retrieve named arm configurations from.
extern const dart::common::Uri namedConfigurationsUri;

class Human final : public aikido::robot::Robot
{
public:
  // Expose base class functions
  using aikido::robot::Robot::getMetaSkeleton;
  using aikido::robot::Robot::getStateSpace;

  /// Construct the Human model.
  ///
  /// \param[in] env World (either for planning, post-processing, or executing)
  /// \param[in] rngSeed seed for initializing random generator
  ///        May be nullptr if simulation is true
  /// \param[in] humanUrdfUri Path to Human URDF model.
  /// \param[in] retriever Resource retriever for retrieving human.
  Human(
      aikido::planner::WorldPtr env,
      aikido::common::RNG::result_type rngSeed = std::random_device{}(),
      const dart::common::Uri& humanUrdfUri = humanUrdfUri,
      const dart::common::ResourceRetrieverPtr& retriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>());

  virtual ~Human() = default;

  // Documentation inherited.
  std::future<void> executeTrajectory(
      const aikido::trajectory::TrajectoryPtr& trajectory) const override;

  // Documentation inherited.
  boost::optional<Eigen::VectorXd> getNamedConfiguration(
      const std::string& name) const override;

  // Documentation inherited.
  void setNamedConfigurations(
      std::unordered_map<std::string, const Eigen::VectorXd>
          namedConfigurations) override;

  // Documentation inherited.
  std::string getName() const override;

  // Documentation inherited.
  dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const override;

  // Documentation inherited.
  aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr getStateSpace()
      const override;

  // Documentation inherited.
  void setRoot(Robot* robot) override;

  // Documentation inherited.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  // Documentation inherited.
  aikido::constraint::dart::CollisionFreePtr getSelfCollisionConstraint(
      const aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const override;

  // Documentation inherited.
  aikido::constraint::TestablePtr getFullCollisionConstraint(
      const aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree)
      const override;

  // Clones RNG
  std::unique_ptr<aikido::common::RNG> cloneRNG();

  /// Get the world
  aikido::planner::WorldPtr getWorld();

  /// Get the right arm
  dart::dynamics::MetaSkeletonPtr getRightArm();

  /// Get the left arm
  dart::dynamics::MetaSkeletonPtr getLeftArm();

  /// Get the right hand
  dart::dynamics::BodyNodePtr getRightHand();

  /// Get the left hand
  dart::dynamics::BodyNodePtr getLeftHand();

  /// Compute IK with left arm.
  std::vector<std::pair<Eigen::VectorXd, double>> computeLeftIK(
    const Eigen::Isometry3d& target,
    const int numSol);

  // Compute IK with right arm.
  std::vector<std::pair<Eigen::VectorXd, double>> computeRightIK(
    const Eigen::Isometry3d& target,
    const int numSol);

  /// Sample a TSR with left arm.
  std::vector<std::pair<Eigen::VectorXd, double>> sampleLeftTSR(
    std::shared_ptr<aikido::constraint::dart::TSR>& tsr,
    const int numSamples);

  /// Sample a TSR with right arm.
  std::vector<std::pair<Eigen::VectorXd, double>> sampleRightTSR(
    std::shared_ptr<aikido::constraint::dart::TSR>& tsr,
    const int numSamples);

  // Set the placement of the human in the plane.
  void setPlacementXYZ(const Eigen::Vector3d& placement);

  // Set the full pose of the human.
  void setPlacementPose(const Eigen::Isometry3d& pose);

private:
  /// Schema description for named configurations YAML file.
  ///
  /// Maps a configuration name (string) to a configuration
  using ConfigurationMap = std::unordered_map<std::string, Eigen::VectorXd>;

  /// Initialize an arm.
  ///
  /// \param[in] armName Name of the arm, either "left" or "right"
  /// \param[in] retriever Resource retriever to resolve URIs
  void configureArm(
      const std::string& armName,
      const dart::common::ResourceRetrieverPtr& retriever);

  // Private helper for common IK logic between left/right arm.
  std::vector<std::pair<Eigen::VectorXd, double>> computeIK(
    const Eigen::Isometry3d& target,
    const int numSol,
    const dart::dynamics::InverseKinematicsPtr& ik,
    const std::shared_ptr<aikido::constraint::Sampleable>& ikSeedSampler,
    const dart::dynamics::MetaSkeletonPtr& arm,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& armSpace,
    const dart::dynamics::BodyNodePtr& hand);

  // Helper for the above for just a single solution. Returns the solution along
  // with its SE(3) error.
  std::pair<Eigen::VectorXd, double> computeSingleIK(
    const Eigen::Isometry3d& target,
    const dart::dynamics::InverseKinematicsPtr& ik,
    const std::shared_ptr<aikido::constraint::SampleGenerator>& ikSeedGenerator,
    const dart::dynamics::MetaSkeletonPtr& arm,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& armSpace,
    const dart::dynamics::BodyNodePtr& hand);

  // Sample IK from the given TSR. Helper used for this between left/right arms.
  std::vector<std::pair<Eigen::VectorXd, double>> sampleTSR(
    std::shared_ptr<aikido::constraint::dart::TSR>& tsr,
    const int numSamples,
    const dart::dynamics::InverseKinematicsPtr& ik,
    const std::shared_ptr<aikido::constraint::Sampleable>& ikSeedSampler,
    const dart::dynamics::MetaSkeletonPtr& arm,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& armSpace,
    const dart::dynamics::BodyNodePtr& hand);

  // Helper that in-place sorts IK solutions on their pose error.
  void sortSolutionsByError(
    std::vector<std::pair<Eigen::VectorXd, double>>& solutionsAndErrors);

  // Correction transform to place human "right side up".
  Eigen::Isometry3d mCorrectionTransform;

  /// Random generator
  aikido::common::RNGWrapper<std::mt19937> mRng;

  /// World that human is in.
  aikido::planner::WorldPtr mWorld;

  /// Underlying robot skeleton
  dart::dynamics::SkeletonPtr mRobotSkeleton;

  /// Robot planning state space
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mSpace;

  // MetaSkeleton for human's torso. Used mainly for IK collision checking.
  dart::dynamics::MetaSkeletonPtr mTorso;

  /// Human's left arm
  dart::dynamics::MetaSkeletonPtr mLeftArm;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mLeftArmSpace;
  dart::dynamics::BodyNodePtr mLeftHand;
  dart::dynamics::InverseKinematicsPtr mLeftIk;

  /// Human's right arm
  dart::dynamics::MetaSkeletonPtr mRightArm;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mRightArmSpace;
  dart::dynamics::BodyNodePtr mRightHand;
  dart::dynamics::InverseKinematicsPtr mRightIk;

  /// Human concrete robot
  aikido::robot::ConcreteRobotPtr mRobot;
};

} // namespace human

#endif // LIBHUMAN_HUMAN_HPP_
