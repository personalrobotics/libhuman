#include "Human.hpp"

#include <cassert>
#include <mutex>
#include <stdexcept>
#include <string>

#include <aikido/common/RNG.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include <aikido/io/yaml.hpp>
#include <aikido/robot/ConcreteManipulator.hpp>
#include <aikido/robot/ConcreteRobot.hpp>
#include <aikido/robot/util.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <dart/common/Timer.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <urdf/model.h>

namespace human {

using dart::dynamics::SkeletonPtr;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::InverseKinematics;
using dart::dynamics::InverseKinematicsPtr;

using aikido::constraint::dart::CollisionFreePtr;
using aikido::constraint::dart::createSampleableBounds;
using aikido::constraint::Sampleable;
using aikido::constraint::SampleGenerator;
using aikido::constraint::TestablePtr;
using aikido::robot::ConcreteManipulatorPtr;
using aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr;
using aikido::statespace::dart::MetaSkeletonStateSaver;
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

double computeSE3Distance(
  const Eigen::Isometry3d& firstPose,
  const Eigen::Isometry3d& secondPose
) {
  double conversionRatioFromRadiusToMeter = 0.17;

  // Borrowed from VFP, should do the same thing as PrPy.
  return aikido::planner::vectorfield::computeGeodesicDistance(
    firstPose, secondPose, conversionRatioFromRadiusToMeter);
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

    // NOTE: Correction so dude is right-side up.
    mCorrectionTransform = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
    mCorrectionTransform.linear() = rot;

    dynamic_cast<dart::dynamics::FreeJoint*>(mRobotSkeleton->getJoint(0))
      ->setTransform(mCorrectionTransform);

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

  // TODO: Enable this again.
  // // Load the named configurations
  // auto namedConfigurations = parseYAMLToNamedConfigurations(
  //     aikido::io::loadYAML(namedConfigurationsUri, retriever));
  // mRobot->setNamedConfigurations(namedConfigurations);

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

std::vector<std::pair<Eigen::VectorXd, double>> Human::computeLeftIK(
  const Eigen::Isometry3d& target,
  const int numSol)
{
  std::shared_ptr<Sampleable> ikSeedSampler
      = createSampleableBounds(mLeftArmSpace, cloneRNG());

  return computeIK(
    target,
    numSol,
    mLeftIk,
    ikSeedSampler,
    mLeftArm,
    mLeftArmSpace,
    mLeftHand);
}

//==============================================================================

std::vector<std::pair<Eigen::VectorXd, double>> Human::computeRightIK(
  const Eigen::Isometry3d& target,
  const int numSol)
{
  std::shared_ptr<Sampleable> ikSeedSampler
      = createSampleableBounds(mRightArmSpace, cloneRNG());

  return computeIK(
    target,
    numSol,
    mRightIk,
    ikSeedSampler,
    mRightArm,
    mRightArmSpace,
    mRightHand);
}

//==============================================================================

std::vector<std::pair<Eigen::VectorXd, double>> Human::sampleLeftTSR(
  std::shared_ptr<aikido::constraint::dart::TSR>& tsr,
  const int numSamples
) {
  std::shared_ptr<Sampleable> ikSeedSampler
      = createSampleableBounds(mLeftArmSpace, cloneRNG());

  return sampleTSR(
    tsr,
    numSamples,
    mLeftIk,
    ikSeedSampler,
    mLeftArm,
    mLeftArmSpace,
    mLeftHand);
}

//==============================================================================

std::vector<std::pair<Eigen::VectorXd, double>> Human::sampleRightTSR(
  std::shared_ptr<aikido::constraint::dart::TSR>& tsr,
  const int numSamples
) {
  std::shared_ptr<Sampleable> ikSeedSampler
      = createSampleableBounds(mRightArmSpace, cloneRNG());

  return sampleTSR(
    tsr,
    numSamples,
    mRightIk,
    ikSeedSampler,
    mRightArm,
    mRightArmSpace,
    mRightHand);
}

//==============================================================================

void Human::setPlacementXYZ(const Eigen::Vector3d& placement)
{
  Eigen::Isometry3d placementTransform = Eigen::Isometry3d::Identity();
  placementTransform.translation() = placement;

  setPlacementPose(placementTransform);
}

//==============================================================================

void Human::setPlacementPose(const Eigen::Isometry3d& pose)
{
  dynamic_cast<dart::dynamics::FreeJoint*>(mRobotSkeleton->getJoint(0))
    ->setTransform(pose * mCorrectionTransform);
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

  // Grab the hand.
  std::stringstream handName;
  handName << armName << "Hand3";
  auto handNode = getBodyNodeOrThrow(mRobotSkeleton, handName.str());

  // Create an IK solver.
  InverseKinematicsPtr ikSolver = InverseKinematics::create(handNode);
  ikSolver->setDofs(arm->getDofs());

  if (armName == "L") {
    mLeftArm = arm;
    mLeftArmSpace = armSpace;
    mLeftHand = handNode;
    mLeftIk = ikSolver;
  } else if (armName == "R") {
    mRightArm = arm;
    mRightArmSpace = armSpace;
    mRightHand = handNode;
    mRightIk = ikSolver;
  } else {
    std::stringstream message;
    message << "configureArm: armName not recognized!";
    throw std::runtime_error(message.str());
  }

  std::cout << "Loaded " << armName << " Arm" << std::endl;
}

//==============================================================================

std::vector<std::pair<Eigen::VectorXd, double>> Human::computeIK(
  const Eigen::Isometry3d& target,
  const int numSol,
  const InverseKinematicsPtr& ik,
  const std::shared_ptr<Sampleable>& ikSeedSampler,
  const dart::dynamics::MetaSkeletonPtr& arm,
  const aikido::statespace::dart::MetaSkeletonStateSpacePtr& armSpace,
  const BodyNodePtr& hand
) {
  auto saver = MetaSkeletonStateSaver(
    arm, MetaSkeletonStateSaver::Options::POSITIONS);
  DART_UNUSED(saver);

  std::vector<std::pair<Eigen::VectorXd, double>> solutionsAndErrors;

  std::shared_ptr<SampleGenerator> ikSeedGenerator
      = ikSeedSampler->createSampleGenerator();

  for (int i = 0; i < numSol; i++)
  {
    std::pair<Eigen::VectorXd, double> solWithError = computeSingleIK(
      target, ik, ikSeedGenerator, arm, armSpace, hand);
    solutionsAndErrors.push_back(solWithError);
  }

  // Ranks IK solutions by final pose error.
  sortSolutionsByError(solutionsAndErrors);

  return solutionsAndErrors;
}

//==============================================================================

std::pair<Eigen::VectorXd, double> Human::computeSingleIK(
  const Eigen::Isometry3d& target,
  const InverseKinematicsPtr& ik,
  const std::shared_ptr<SampleGenerator>& ikSeedGenerator,
  const dart::dynamics::MetaSkeletonPtr& arm,
  const aikido::statespace::dart::MetaSkeletonStateSpacePtr& armSpace,
  const BodyNodePtr& hand
) {
  auto seedState = armSpace->createState();

  if (!ikSeedGenerator->sample(seedState))
  {
    std::stringstream message;
    message << "computeSingleIK: out of seed configs!";
    throw std::runtime_error(message.str());
  }

  armSpace->setState(arm.get(), seedState);
  ik->getTarget()->setTransform(target);
  ik->solve(true);
  Eigen::VectorXd curSol = arm->getPositions();

  double curError = computeSE3Distance(hand->getTransform(), target);
  return std::make_pair(curSol, curError);
}

//==============================================================================

std::vector<std::pair<Eigen::VectorXd, double>> Human::sampleTSR(
  std::shared_ptr<aikido::constraint::dart::TSR>& tsr,
  const int numSamples,
  const InverseKinematicsPtr& ik,
  const std::shared_ptr<Sampleable>& ikSeedSampler,
  const dart::dynamics::MetaSkeletonPtr& arm,
  const aikido::statespace::dart::MetaSkeletonStateSpacePtr& armSpace,
  const BodyNodePtr& hand
) {
  std::shared_ptr<SampleGenerator> ikSeedGenerator
      = ikSeedSampler->createSampleGenerator();

  tsr->setRNG(cloneRNG());

  // Used to actually sample poses.
  std::unique_ptr<SampleGenerator> poseSampler = tsr->createSampleGenerator();

  if (poseSampler->getNumSamples() != SampleGenerator::NO_LIMIT)
  {
    throw std::invalid_argument("sampleTSR: TSR does not have inf samples!");
  }

  using aikido::statespace::SE3;
  std::shared_ptr<const SE3> poseStateSpace
    = std::dynamic_pointer_cast<const SE3>(poseSampler->getStateSpace());
  if (!poseStateSpace)
    throw std::invalid_argument("sampleTSR: TSR does not operate on SE3!");

  std::vector<std::pair<Eigen::VectorXd, double>> samplesAndErrors;
  for (int poseIndex = 0; poseIndex < numSamples; poseIndex++)
  {
    auto poseState = poseStateSpace->createState();
    // Assert that we can still sample poses.
    if (!poseSampler->sample(poseState))
      throw std::runtime_error("sampleTSR: Pose sampler exhausted!");

    Eigen::Isometry3d curTargetPose = poseState.getIsometry();
    std::pair<Eigen::VectorXd, double> sampleWithError = computeSingleIK(
      curTargetPose, ik, ikSeedGenerator, arm, armSpace, hand);

    samplesAndErrors.push_back(sampleWithError);
  }

  // Ranks IK solutions by final pose error.
  sortSolutionsByError(samplesAndErrors);

  return samplesAndErrors;
}

//==============================================================================

void Human::sortSolutionsByError(
  std::vector<std::pair<Eigen::VectorXd, double>>& solutionsAndErrors
) {
  auto sortByError =
      [](const std::pair<Eigen::VectorXd, double>& a,
        const std::pair<Eigen::VectorXd, double>& b) {
        return a.second < b.second;
      };
  std::sort(solutionsAndErrors.begin(), solutionsAndErrors.end(), sortByError);
}

} // ns
