#include "Human.hpp"

#include <cassert>
#include <mutex>
#include <stdexcept>
#include <string>

#include <aikido/control/KinematicSimulationTrajectoryExecutor.hpp>
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
#include <dart/utils/urdf/urdf.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>

#undef dtwarn
#define dtwarn (::dart::common::colorErr("Warning", __FILE__, __LINE__, 33))

#undef dtinfo
#define dtinfo (::dart::common::colorMsg("Info", 32))

namespace human {

using dart::collision::CollisionGroup;
using dart::dynamics::Chain;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::InverseKinematics;
using dart::dynamics::InverseKinematicsPtr;

using aikido::constraint::dart::CollisionFree;
using aikido::constraint::dart::CollisionFreePtr;
using aikido::constraint::dart::createSampleableBounds;
using aikido::constraint::dart::TSRPtr;
using aikido::constraint::Sampleable;
using aikido::constraint::SampleGenerator;
using aikido::constraint::TestablePtr;
using aikido::robot::ConcreteManipulatorPtr;
using aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr;
using aikido::statespace::dart::MetaSkeletonStateSaver;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::trajectory::TrajectoryPtr;
using aikido::trajectory::UniqueSplinePtr;

dart::common::Uri defaultPRLUrdfUri{"package://libhuman/robot/man1.urdf"};
dart::common::Uri defaultPRLSrdfUri{"package://libhuman/robot/man1.srdf"};
dart::common::Uri defaultVisPRLUrdfUri{"package://libhuman/robot/vis_man1.urdf"};
dart::common::Uri defaultVisPRLSrdfUri{"package://libhuman/robot/vis_man1.srdf"};
dart::common::Uri defaultICAROSUrdfUri{"package://libhuman/robot/human_short.urdf"};
dart::common::Uri defaultICAROSSrdfUri{"package://libhuman/robot/human_short.srdf"};
dart::common::Uri defaultVisICAROSUrdfUri{"package://libhuman/robot/vis_human_short.urdf"};
dart::common::Uri defaultVisICAROSSrdfUri{"package://libhuman/robot/vis_human_short.srdf"};
const dart::common::Uri namedConfigurationsUri{"TODO"};

// Arm trajectory controllers that are meant to be used by the human.
// Needs to be consistent with the configurations in human_launch.
// Right now there is no such a thing.
const std::vector<std::string> availableArmTrajectoryExecutorNames{
    "trajectory_controller",
    "rewd_trajectory_controller",
    "move_until_touch_topic_controller"};

namespace {
BodyNodePtr getBodyNodeOrThrow(
    const SkeletonPtr &skeleton, const std::string &bodyNodeName) {
  auto bodyNode = skeleton->getBodyNode(bodyNodeName);

  if (!bodyNode) {
    std::stringstream message;
    message << "Bodynode [" << bodyNodeName << "] does not exist in skeleton.";
    throw std::runtime_error(message.str());
  }

  return bodyNode;
}

double computeSE3Distance(
    const Eigen::Isometry3d &firstPose,
    const Eigen::Isometry3d &secondPose
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
    bool simulation,
    std::string modelSrc,
    aikido::common::RNG::result_type rngSeed,
    const std::string &endEffectorName,
    const std::string &armTrajectoryExecutorName,
    const ::ros::NodeHandle *node,
    const dart::common::ResourceRetrieverPtr &retriever)
    : mSimulation(simulation),
      mArmTrajectoryExecutorName(armTrajectoryExecutorName),
      mRng(rngSeed),
      mWorld(std::move(env)),
      mEndEffectorName(endEffectorName) {
  if (std::find(availableArmTrajectoryExecutorNames.begin(),
                availableArmTrajectoryExecutorNames.end(),
                mArmTrajectoryExecutorName) == availableArmTrajectoryExecutorNames.end()) {
    throw std::runtime_error("Arm Trajectory Controller is not valid!");
  }

  dtinfo << "Arm Executor " << armTrajectoryExecutorName << std::endl;

  std::string name = "man1";

  dart::common::Uri humanUrdfUri;
  dart::common::Uri humanSrdfUri;

  // Given the different model sources, we use different urdf files
  if (modelSrc == "icaros") {
    humanUrdfUri = defaultICAROSUrdfUri;
    humanSrdfUri = defaultICAROSSrdfUri;
  } else if (modelSrc == "prl") {
    humanUrdfUri = defaultPRLUrdfUri;
    humanSrdfUri = defaultPRLSrdfUri;
  } else {
    throw std::runtime_error("Given model source " + modelSrc + " is not valid! ");
  }

  // Load Human.
  mRobotSkeleton = mWorld->getSkeleton(name); // TODO(bhou): set as constant
  if (!mRobotSkeleton) {
    /*dart::utils::DartLoader urdfLoader;
    mRobotSkeleton = urdfLoader.parseSkeleton(humanUrdfUri, retriever);

    // NOTE: Correction so dude is right-side up.
    mCorrectionTransform = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
    
    mCorrectionTransform.linear() = rot;*/
    dart::utils::DartLoader urdfLoader;

    mCorrectionTransform = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rot;

    // there are two different human urdf files.
    if (modelSrc == "icaros") {
      mRobotSkeleton = urdfLoader.parseSkeleton(humanUrdfUri, retriever);
      rot = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    } else if (modelSrc == "prl") {
      mRobotSkeleton = urdfLoader.parseSkeleton(humanUrdfUri, retriever);
      rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
    }

    mCorrectionTransform.linear() = rot;

    dynamic_cast<dart::dynamics::FreeJoint *>(mRobotSkeleton->getJoint(0))
        ->setTransform(mCorrectionTransform);

    mWorld->addSkeleton(mRobotSkeleton);
  }

  if (!mRobotSkeleton) {
    throw std::runtime_error("Unable to load Human model.");
  }

  // Define the collision detector and groups
  auto collisionDetector = dart::collision::FCLCollisionDetector::create();
  auto selfCollisionFilter = std::make_shared<dart::collision::BodyNodeCollisionFilter>();

  urdf::Model urdfModel;
  std::string humanUrdfXMLString = retriever->readAll(humanUrdfUri);
  urdfModel.initString(humanUrdfXMLString);

  srdf::Model srdfModel;
  std::string humanSrdfXMLString = retriever->readAll(humanSrdfUri);
  srdfModel.initString(urdfModel, humanSrdfXMLString);
  auto disabledCollisions = srdfModel.getDisabledCollisionPairs();

  for (auto disabledPair : disabledCollisions) {
    auto body0 = getBodyNodeOrThrow(mRobotSkeleton, disabledPair.link1_);
    auto body1 = getBodyNodeOrThrow(mRobotSkeleton, disabledPair.link2_);

#ifndef NDEBUG
    dtinfo << "Disabled collisions between " << disabledPair.link1_ << " and " << disabledPair.link2_ << std::endl;
#endif

    selfCollisionFilter->addBodyNodePairToBlackList(body0, body1);
  }

  mSpace = std::make_shared<MetaSkeletonStateSpace>(mRobotSkeleton.get());

  mTrajectoryExecutor = createTrajectoryExecutor();

  // Setting arm base and end names
  mArmBaseName = "RCollar";

  if (modelSrc == "icaros") {
      mArmEndName = "RWrist";
  } else if (modelSrc == "prl") {
      mArmEndName = "RHand2";
  }

  mHandBaseName = "RHand3";

  // Setup the arm
  mRightArm = configureRightArm(
      "R", //human_right_arm
      retriever,
      mTrajectoryExecutor,
      collisionDetector,
      selfCollisionFilter);

  mRightArmSpace = mRightArm->getStateSpace();

  // Set up the concrete robot from the meta skeleton
  mHuman = std::make_shared<aikido::robot::ConcreteRobot>(
      "Human",
      mRobotSkeleton,
      mSimulation,
      cloneRNG(),
      mTrajectoryExecutor,
      collisionDetector,
      selfCollisionFilter);

  // Used for IK collision checking.
  BodyNodePtr hipsNode = getBodyNodeOrThrow(mRobotSkeleton, "Hips");
  BodyNodePtr collarNode = getBodyNodeOrThrow(mRobotSkeleton, "RCollar");
  mTorso = Chain::create(hipsNode, collarNode, "Torso");

  // TODO: Enable this.
  configureArm("L", retriever);
//  configureArm("R", retriever);  commented out for now

  // TODO: Enable this again.
  // // Load the named configurations
  // auto namedConfigurations = parseYAMLToNamedConfigurations(
  //     aikido::io::loadYAML(namedConfigurationsUri, retriever));
  // mRobot->setNamedConfigurations(namedConfigurations);

  // TODO!

  mThread = std::make_unique<aikido::common::ExecutorThread>(std::bind(&Human::update, this),
                                                             threadExecutionCycle);
}

//===================================================================================================================
Human::Human(aikido::planner::WorldPtr env,
             bool simulation,
             std::string name,
             const Eigen::Isometry3d &transform,
             bool vis,
             std::string modelSrc,
             aikido::common::RNG::result_type rngSeed,
             const std::string &endEffectorName,
             const std::string &armTrajectoryExecutorName,
             const ::ros::NodeHandle *node,
             const dart::common::ResourceRetrieverPtr &retriever) :
    mSimulation(simulation),
    mArmTrajectoryExecutorName(armTrajectoryExecutorName),
    mRng(rngSeed),
    mWorld(std::move(env)),
    mEndEffectorName(endEffectorName) {
  if (std::find(availableArmTrajectoryExecutorNames.begin(),
                availableArmTrajectoryExecutorNames.end(),
                mArmTrajectoryExecutorName) == availableArmTrajectoryExecutorNames.end()) {
    throw std::runtime_error("Arm Trajectory Controller is not valid!");
  }

  dtinfo << "Arm Executor " << armTrajectoryExecutorName << std::endl;

  dart::common::Uri humanUrdfUri;
  dart::common::Uri humanSrdfUri;

  if (vis) {
    std::cout << "vis is true" << std::endl;
    // Given the different model sources, we use different urdf files
    if (modelSrc == "icaros") {
      humanUrdfUri = defaultVisICAROSUrdfUri;
      humanSrdfUri = defaultVisICAROSSrdfUri;
    } else if (modelSrc == "prl") {
      humanUrdfUri = defaultVisPRLUrdfUri;
      humanSrdfUri = defaultVisPRLSrdfUri;
    } else {
      throw std::runtime_error("Given model source " + modelSrc + " is not valid!");
    }
  } else {
    // Given the different model sources, we use different urdf files
    if (modelSrc == "icaros") {
      humanUrdfUri = defaultICAROSUrdfUri;
      humanSrdfUri = defaultICAROSSrdfUri;
    } else if (modelSrc == "prl") {
      humanUrdfUri = defaultPRLUrdfUri;
      humanSrdfUri = defaultPRLSrdfUri;
    } else {
      throw std::runtime_error("Given model source " + modelSrc + " is not valid!");
    }
  }

//  std::string name = "man1";

  // Load Human.
  mRobotSkeleton = mWorld->getSkeleton(name); // TODO(bhou): set as constant
  if (!mRobotSkeleton) {
    /*dart::utils::DartLoader urdfLoader;
    mRobotSkeleton = urdfLoader.parseSkeleton(humanUrdfUri, retriever);

    // NOTE: Correction so dude is right-side up.
    mCorrectionTransform = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());

    mCorrectionTransform.linear() = rot;*/
    dart::utils::DartLoader urdfLoader;

    mCorrectionTransform = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rot;

    // there are two different human urdf files.
    if (modelSrc == "icaros") {
      mRobotSkeleton = urdfLoader.parseSkeleton(humanUrdfUri, retriever);
      rot = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    } else if (modelSrc == "prl") {
      mRobotSkeleton = urdfLoader.parseSkeleton(humanUrdfUri, retriever);
      rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
    }

    mCorrectionTransform.linear() = rot;

//    dynamic_cast<dart::dynamics::FreeJoint *>(mRobotSkeleton->getJoint(0))
//        ->setTransform(mCorrectionTransform);

    dynamic_cast<dart::dynamics::FreeJoint *>(mRobotSkeleton->getJoint(0))
        ->setTransform(transform);

    mWorld->addSkeleton(mRobotSkeleton);
  }

  if (!mRobotSkeleton) {
    throw std::runtime_error("Unable to load Human model.");
  }

  // Define the collision detector and groups
  auto collisionDetector = dart::collision::FCLCollisionDetector::create();
  auto selfCollisionFilter = std::make_shared<dart::collision::BodyNodeCollisionFilter>();

  urdf::Model urdfModel;
  std::string humanUrdfXMLString = retriever->readAll(humanUrdfUri);
  urdfModel.initString(humanUrdfXMLString);

  srdf::Model srdfModel;
  std::string humanSrdfXMLString = retriever->readAll(humanSrdfUri);
  srdfModel.initString(urdfModel, humanSrdfXMLString);
  auto disabledCollisions = srdfModel.getDisabledCollisionPairs();

  for (auto disabledPair : disabledCollisions) {
    auto body0 = getBodyNodeOrThrow(mRobotSkeleton, disabledPair.link1_);
    auto body1 = getBodyNodeOrThrow(mRobotSkeleton, disabledPair.link2_);

#ifndef NDEBUG
    dtinfo << "Disabled collisions between " << disabledPair.link1_ << " and " << disabledPair.link2_ << std::endl;
#endif

    selfCollisionFilter->addBodyNodePairToBlackList(body0, body1);
  }

  mSpace = std::make_shared<MetaSkeletonStateSpace>(mRobotSkeleton.get());

  mTrajectoryExecutor = createTrajectoryExecutor();

  // Setting arm base and end names
  mArmBaseName = "RCollar";

  if (modelSrc == "icaros") {
      mArmEndName = "RWrist";
  } else if (modelSrc == "prl") {
      mArmEndName = "RHand2";
  }

  mHandBaseName = "RHand3";

  // Setup the arm
  mRightArm = configureRightArm(
      "R", //human_right_arm
      retriever,
      mTrajectoryExecutor,
      collisionDetector,
      selfCollisionFilter);

  mRightArmSpace = mRightArm->getStateSpace();

  // Set up the concrete robot from the meta skeleton
  mHuman = std::make_shared<aikido::robot::ConcreteRobot>(
      "Human",
      mRobotSkeleton,
      mSimulation,
      cloneRNG(),
      mTrajectoryExecutor,
      collisionDetector,
      selfCollisionFilter);

  // Used for IK collision checking.
  BodyNodePtr hipsNode = getBodyNodeOrThrow(mRobotSkeleton, "Hips");
  BodyNodePtr collarNode = getBodyNodeOrThrow(mRobotSkeleton, "RCollar");
  mTorso = Chain::create(hipsNode, collarNode, "Torso");

  // TODO: Enable this.
  configureArm("L", retriever);
//  configureArm("R", retriever);  commented out for now

  // TODO: Enable this again.
  // // Load the named configurations
  // auto namedConfigurations = parseYAMLToNamedConfigurations(
  //     aikido::io::loadYAML(namedConfigurationsUri, retriever));
  // mRobot->setNamedConfigurations(namedConfigurations);

  // TODO!

  mThread = std::make_unique<aikido::common::ExecutorThread>(std::bind(&Human::update, this),
                                                             threadExecutionCycle);
}

void Human::update() {
  step(std::chrono::system_clock::now());
}

std::shared_ptr<aikido::control::TrajectoryExecutor> Human::createTrajectoryExecutor() {
  if (mSimulation) {
    return std::make_shared<aikido::control::KinematicSimulationTrajectoryExecutor>(mRobotSkeleton);
  }
}

//==============================================================================
std::future<void> Human::executeTrajectory(const TrajectoryPtr &trajectory) const {
  return mTrajectoryExecutor->execute(trajectory);
}

//==============================================================================
boost::optional<Eigen::VectorXd> Human::getNamedConfiguration(
    const std::string &name) const {
  return mHuman->getNamedConfiguration(name);
}

//==============================================================================
void Human::setNamedConfigurations(
    std::unordered_map<std::string, const Eigen::VectorXd> namedConfigurations) {
  mHuman->setNamedConfigurations(namedConfigurations);
}

//==============================================================================
std::string Human::getName() const {
  return mHuman->getName();
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr Human::getMetaSkeleton() const {
  return mHuman->getMetaSkeleton();
}

//==============================================================================
ConstMetaSkeletonStateSpacePtr Human::getStateSpace() const {
  return mHuman->getStateSpace();
}

//==============================================================================
void Human::setRoot(Robot *robot) {
  mHuman->setRoot(robot);
}

//==============================================================================
void Human::step(const std::chrono::system_clock::time_point &timepoint) {
  // TODO!
  std::lock_guard<std::mutex> lock(mRobotSkeleton->getMutex());
  mRightArm->step(timepoint);
  mTrajectoryExecutor->step(timepoint);
}

//==============================================================================
CollisionFreePtr Human::getSelfCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr &space,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton) const {
  // Set up collision constraints.
  auto collDetector = dart::collision::FCLCollisionDetector::create();
  auto collTestable
      = std::make_shared<CollisionFree>(space, metaSkeleton, collDetector);

  std::shared_ptr<CollisionGroup> armGroup =
      collDetector->createCollisionGroup(metaSkeleton.get());

  std::shared_ptr<CollisionGroup> torsoObstacleGroup =
      collDetector->createCollisionGroup(mTorso.get());

  collTestable->addPairwiseCheck(armGroup, torsoObstacleGroup);

  return collTestable;
}

//==============================================================================
TestablePtr Human::getFullCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr &space,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const CollisionFreePtr &collisionFree) const {
  return mHuman->getFullCollisionConstraint(space, metaSkeleton, collisionFree);
}

//==============================================================================
std::unique_ptr<aikido::common::RNG> Human::cloneRNG() {
  return std::move(cloneRNGFrom(mRng)[0]);
}

//==============================================================================
aikido::planner::WorldPtr Human::getWorld() {
  return mWorld;
}

//==============================================================================
aikido::robot::ConcreteManipulatorPtr Human::getRightArm() {
  return mRightArm;
}

//==============================================================================
dart::dynamics::MetaSkeletonPtr Human::getLeftArm() {
  return mLeftArm;
}

//==============================================================================
aikido::statespace::dart::MetaSkeletonStateSpacePtr Human::getRightArmSpace() {
  return mRightArmSpace;
}

//==============================================================================
aikido::statespace::dart::MetaSkeletonStateSpacePtr Human::getLeftArmSpace() {
  return mLeftArmSpace;
}

//==============================================================================
BodyNodePtr Human::getRightHand() {
  // TODO!
  throw std::runtime_error("Human -> getRightHand() not implemented!");
}

//==============================================================================
BodyNodePtr Human::getLeftHand() {
  // TODO!
  throw std::runtime_error("Human -> getLeftHand() not implemented!");
}

//==============================================================================
aikido::robot::ConcreteManipulatorPtr Human::configureRightArm(const std::string &armName,
                                                               const dart::common::ResourceRetrieverPtr &retriever,
                                                               const aikido::control::TrajectoryExecutorPtr &executor,
                                                               dart::collision::CollisionDetectorPtr collisionDetector,
                                                               const std::shared_ptr<dart::collision::BodyNodeCollisionFilter> &selfCollisionFilter) {
  auto armBase = getBodyNodeOrThrow(mRobotSkeleton, mArmBaseName);
  auto armEnd = getBodyNodeOrThrow(mRobotSkeleton, mArmEndName);

  auto arm = dart::dynamics::Chain::create(armBase, armEnd, armName);
  auto armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(arm.get());

  mHand = std::make_shared<HumanHand>(armName, mSimulation,
                                      getBodyNodeOrThrow(mRobotSkeleton, mHandBaseName),
                                      getBodyNodeOrThrow(mRobotSkeleton, mEndEffectorName),
                                      selfCollisionFilter,
                                      mNode.get(),
                                      retriever);

      // Hardcoding to acceleration limits used in OpenRAVE
      // This is necessary because ADA is loaded from URDF, which
      // provides no means of specifying acceleration limits
      // TODO : update acceleration limits by checking Kinova spec.
      arm->setAccelerationLowerLimits(
              Eigen::VectorXd::Constant(arm->getNumDofs(), -2.0));
      arm->setAccelerationUpperLimits(
              Eigen::VectorXd::Constant(arm->getNumDofs(), 2.0));

  auto manipulatorRobot = std::make_shared<aikido::robot::ConcreteRobot>(
      armName,
      arm,
      mSimulation,
      cloneRNG(),
      executor,
      collisionDetector,
      selfCollisionFilter);

  auto manipulator = std::make_shared<aikido::robot::ConcreteManipulator>(manipulatorRobot, mHand);

  //Grab the hand
  std::stringstream handName;
  handName << armName << "Hand3";
  auto handNode = getBodyNodeOrThrow(mRobotSkeleton, handName.str());

  // Create an IK solver.
  InverseKinematicsPtr ikSolver = InverseKinematics::create(handNode);
  ikSolver->setDofs(arm->getDofs());

  if(armName == "R"){
      //mRightArmSpace = armSpace;
      mRightHand = handNode;
      mRightIk = ikSolver;
  }


  return manipulator;
}

//==============================================================================

std::vector<std::pair<Eigen::VectorXd, double>> Human::computeLeftIK(
    const Eigen::Isometry3d &target,
    const int numSol,
    const TestablePtr constraint) {
  std::shared_ptr<Sampleable> ikSeedSampler
      = createSampleableBounds(mLeftArmSpace, cloneRNG());

  return computeIK(
      target,
      numSol,
      mLeftIk,
      ikSeedSampler,
      mLeftArm,
      mLeftArmSpace,
      mLeftHand,
      constraint);
}

//==============================================================================

std::vector<std::pair<Eigen::VectorXd, double>> Human::computeRightIK(
    const Eigen::Isometry3d &target,
    const int numSol,
    const TestablePtr constraint) {
  std::shared_ptr<Sampleable> ikSeedSampler
      = createSampleableBounds(mRightArmSpace, cloneRNG());

  return computeIK(
      target,
      numSol,
      mRightIk,
      ikSeedSampler,
      mRightArm->getMetaSkeleton(),
      mRightArmSpace,
      mRightHand,
      constraint);
}

//==============================================================================
TrajectoryPtr Human::planToTSR(
        const MetaSkeletonStateSpacePtr& space,
        const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
        const dart::dynamics::BodyNodePtr& bn,
        const TSRPtr& tsr,
        const CollisionFreePtr& collisionFree,
        double timelimit,
        size_t maxNumTrials,
        const aikido::distance::ConfigurationRankerPtr& ranker)
    {

        return mHuman->planToTSR(
                space,
                metaSkeleton,
                bn,
                tsr,
                collisionFree,
                timelimit,
                maxNumTrials,
                ranker);
    }

//==============================================================================
TrajectoryPtr Human::planRightArmToTSR(
        std::shared_ptr<aikido::constraint::dart::TSR> &tsr,    //const aikido::constraint::dart::TSR& tsr,
        const aikido::constraint::dart::CollisionFreePtr& collisionFree,
        double timelimit,
        size_t maxNumTrials,
        const aikido::distance::ConfigurationRankerPtr& ranker)


    {
        //auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(tsr);
        std::cout<<"work"<<std::endl;
        return planToTSR(
                mRightArmSpace,
                mRightArm->getMetaSkeleton(),
                mHand->getEndEffectorBodyNode(),
                tsr,    //goalTSR,
                collisionFree,
                timelimit,
                maxNumTrials,
                ranker);
    }

//==============================================================================
    Eigen::VectorXd Human::getVelocityLimits() const
    {
        return mRightArm->getMetaSkeleton()->getVelocityUpperLimits();
    }

//==============================================================================
    Eigen::VectorXd Human::getAccelerationLimits() const
    {
        return mRightArm->getMetaSkeleton()->getAccelerationUpperLimits();
    }
//==============================================================================

std::vector<std::pair<Eigen::VectorXd, double>> Human::sampleLeftTSR(
    std::shared_ptr<aikido::constraint::dart::TSR> &tsr,
    const int numSamples,
    const TestablePtr constraint
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
      mLeftHand,
      constraint);
}

//==============================================================================

std::vector<std::pair<Eigen::VectorXd, double>> Human::sampleRightTSR(
    std::shared_ptr<aikido::constraint::dart::TSR> &tsr,
    const int numSamples,
    const TestablePtr constraint
) {
  std::shared_ptr<Sampleable> ikSeedSampler
      = createSampleableBounds(mRightArmSpace, cloneRNG());

  return sampleTSR(
      tsr,
      numSamples,
      mRightIk,
      ikSeedSampler,
      mRightArm->getMetaSkeleton(),
      mRightArmSpace,
      mRightHand,
      constraint);
}

//==============================================================================

void Human::setPlacementXYZ(const Eigen::Vector3d &placement) {
  Eigen::Isometry3d placementTransform = Eigen::Isometry3d::Identity();
  placementTransform.translation() = placement;

  setPlacementPose(placementTransform);
}

//==============================================================================

void Human::setPlacementPose(const Eigen::Isometry3d &pose) {
  dynamic_cast<dart::dynamics::FreeJoint *>(mRobotSkeleton->getJoint(0))
      ->setTransform(pose * mCorrectionTransform);
}

//==============================================================================
void Human::configureArm(
    const std::string &armName,
    const dart::common::ResourceRetrieverPtr &retriever) {
  std::stringstream armStartName;
  armStartName << armName << "Collar";

  std::stringstream armEndName;
  armEndName << armName << "Hand3";

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
    // no R
//    mRightArm = arm;
//    mRightArmSpace = armSpace;
//    mRightHand = handNode;
//    mRightIk = ikSolver;
  } else {
    std::stringstream message;
    message << "configureArm: armName not recognized!";
    throw std::runtime_error(message.str());
  }

  std::cout << "Loaded " << armName << " Arm" << std::endl;
}

//==============================================================================

std::vector<std::pair<Eigen::VectorXd, double>> Human::computeIK(
    const Eigen::Isometry3d &target,
    const int numSol,
    const InverseKinematicsPtr &ik,
    const std::shared_ptr<Sampleable> &ikSeedSampler,
    const dart::dynamics::MetaSkeletonPtr &arm,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr &armSpace,
    const BodyNodePtr &hand,
    const TestablePtr constraint
) {
  auto saver = MetaSkeletonStateSaver(
      arm, MetaSkeletonStateSaver::Options::POSITIONS);
  DART_UNUSED(saver);

  std::vector<std::pair<Eigen::VectorXd, double>> solutionsAndErrors;

  std::shared_ptr<SampleGenerator> ikSeedGenerator
      = ikSeedSampler->createSampleGenerator();

  for (int i = 0; i < numSol; i++) {
    std::pair<Eigen::VectorXd, double> solWithError = computeSingleIK(
        target, ik, ikSeedGenerator, arm, armSpace, hand);
    solutionsAndErrors.push_back(solWithError);
  }

  // Ranks IK solutions by final pose error.
  filterSortSolutions(solutionsAndErrors, constraint, armSpace);

  return solutionsAndErrors;
}

//==============================================================================

std::pair<Eigen::VectorXd, double> Human::computeSingleIK(
    const Eigen::Isometry3d &target,
    const InverseKinematicsPtr &ik,
    const std::shared_ptr<SampleGenerator> &ikSeedGenerator,
    const dart::dynamics::MetaSkeletonPtr &arm,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr &armSpace,
    const BodyNodePtr &hand
) {
  auto seedState = armSpace->createState();

  if (!ikSeedGenerator->sample(seedState)) {
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
    std::shared_ptr<aikido::constraint::dart::TSR> &tsr,
    const int numSamples,
    const InverseKinematicsPtr &ik,
    const std::shared_ptr<Sampleable> &ikSeedSampler,
    const dart::dynamics::MetaSkeletonPtr &arm,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr &armSpace,
    const BodyNodePtr &hand,
    const TestablePtr constraint
) {
  std::shared_ptr<SampleGenerator> ikSeedGenerator
      = ikSeedSampler->createSampleGenerator();

  tsr->setRNG(cloneRNG());

  // Used to actually sample poses.
  std::unique_ptr<SampleGenerator> poseSampler = tsr->createSampleGenerator();

  if (poseSampler->getNumSamples() != SampleGenerator::NO_LIMIT) {
    throw std::invalid_argument("sampleTSR: TSR does not have inf samples!");
  }

  using aikido::statespace::SE3;
  std::shared_ptr<const SE3> poseStateSpace
      = std::dynamic_pointer_cast<const SE3>(poseSampler->getStateSpace());
  if (!poseStateSpace)
    throw std::invalid_argument("sampleTSR: TSR does not operate on SE3!");

  std::vector<std::pair<Eigen::VectorXd, double>> samplesAndErrors;
  for (int poseIndex = 0; poseIndex < numSamples; poseIndex++) {
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
  filterSortSolutions(samplesAndErrors, constraint, armSpace);

  return samplesAndErrors;
}

//==============================================================================

void Human::filterSortSolutions(
    std::vector<std::pair<Eigen::VectorXd, double>> &solutionsAndErrors,
    const TestablePtr constraint,
    aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace
) {
  auto sortByError =
      [](const std::pair<Eigen::VectorXd, double> &a,
         const std::pair<Eigen::VectorXd, double> &b) {
        return a.second < b.second;
      };
  std::sort(solutionsAndErrors.begin(), solutionsAndErrors.end(), sortByError);

  // Also filter using `constraint` (if given).
  if (constraint && stateSpace) {
    auto testState = stateSpace->createState();

    std::vector<std::pair<Eigen::VectorXd, double>> filteredSols;
    for (auto &curPair : solutionsAndErrors) {
      Eigen::VectorXd curConfig = curPair.first;
      stateSpace->convertPositionsToState(curConfig, testState);

      if (constraint->isSatisfied(testState)) {
        filteredSols.push_back(curPair);
      }
    }

    solutionsAndErrors = filteredSols;
  }
}

} // ns

