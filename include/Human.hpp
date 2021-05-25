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
  aikido::robot::ConcreteManipulatorPtr getRightArm();

  /// Get the left arm
  aikido::robot::ConcreteManipulatorPtr getLeftArm();

  /// Get the right hand
  dart::dynamics::BodyNodePtr getRightHand();

  /// Get the left hand
  dart::dynamics::BodyNodePtr getLeftHand();

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

  /// Random generator
  aikido::common::RNGWrapper<std::mt19937> mRng;

  /// World that human is in.
  aikido::planner::WorldPtr mWorld;

  /// Underlying robot skeleton
  dart::dynamics::SkeletonPtr mRobotSkeleton;

  /// Robot planning state space
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mSpace;

  /// Human's left arm
  dart::dynamics::MetaSkeletonPtr mLeftArm;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mLeftArmSpace;

  /// Human's right arm
  dart::dynamics::MetaSkeletonPtr mRightArm;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mRightArmSpace;

  /// Human concrete robot
  aikido::robot::ConcreteRobotPtr mRobot;
};

} // namespace human

#endif // LIBHUMAN_HUMAN_HPP_
