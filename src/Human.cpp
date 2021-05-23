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
  using aikido::common::ExecutorThread;
  using aikido::control::ros::RosJointStateClient;

  std::string name = "man1";

  // Load Human.
  mRobotSkeleton = mWorld->getSkeleton(name); // TODO(bhou): set as constant
  if (!mRobotSkeleton)
  {
    dart::utils::DartLoader urdfLoader;
    mRobotSkeleton = urdfLoader.parseSkeleton(herbUrdfUri, retriever);
    mWorld->addSkeleton(mRobotSkeleton);
  }

  if (!mRobotSkeleton)
  {
    throw std::runtime_error("Unable to load HERB model.");
  }

  if (!mSimulation)
  {
    if (!node)
    {
      mNode = dart::common::make_unique<::ros::NodeHandle>();
    }
    else
    {
      mNode = dart::common::make_unique<::ros::NodeHandle>(*node);
    }

    mControllerServiceClient = dart::common::make_unique<::ros::ServiceClient>(
        mNode->serviceClient<controller_manager_msgs::SwitchController>(
            "controller_manager/switch_controller"));

    mTalkerActionClient = dart::common::
        make_unique<actionlib::SimpleActionClient<talker::SayAction>>(
            "say", true);

    mJointStateClient = dart::common::make_unique<RosJointStateClient>(
        mRobotSkeleton, *mNode, "/joint_states", 1);
    mJointStateThread = dart::common::make_unique<ExecutorThread>(
        std::bind(&RosJointStateClient::spin, mJointStateClient.get()),
        mRosParams.mJointStateUpdatePeriod);
    ros::Duration(0.3).sleep(); // first callback at around 0.12 - 0.25 seconds
  }

  auto collisionDetector = FCLCollisionDetector::create();
  auto collideWith = collisionDetector->createCollisionGroupAsSharedPtr();
  auto selfCollisionFilter
      = std::make_shared<dart::collision::BodyNodeCollisionFilter>();

  // Load in the SRDF and disable collision checking between the pairs
  // of bodies that it specifies.
  // TODO: Avoid loading the URDF this second time.
  // See: https://github.com/personalrobotics/libherb/pull/63.
  urdf::Model urdfModel;
  std::string herbUrdfXMLString = retriever->readAll(herbUrdfUri);
  urdfModel.initString(herbUrdfXMLString);

  srdf::Model srdfModel;
  std::string herbSrdfXMLString = retriever->readAll(herbSrdfUri);
  srdfModel.initString(urdfModel, herbSrdfXMLString);
  auto disabledCollisions = srdfModel.getDisabledCollisionPairs();

  for (auto disabledPair : disabledCollisions)
  {
    auto body0 = getBodyNodeOrThrow(mRobotSkeleton, disabledPair.link1_);
    auto body1 = getBodyNodeOrThrow(mRobotSkeleton, disabledPair.link2_);

#ifndef NDEBUG
    std::cout << "[INFO] Disabled collisions between " << disabledPair.link1_
              << " and " << disabledPair.link2_ << std::endl;
#endif

    selfCollisionFilter->addBodyNodePairToBlackList(body0, body1);
  }

  mSpace = std::make_shared<MetaSkeletonStateSpace>(mRobotSkeleton.get());

  mTrajectoryExecutor = createTrajectoryExecutor();
  mNeckPositionExecutor = createNeckPositionExecutor(configureNeck());

  // Setup both arms
  mLeftArm
      = configureArm("left", retriever, collisionDetector, selfCollisionFilter);
  mRightArm = configureArm(
      "right", retriever, collisionDetector, selfCollisionFilter);

  // Setup robot
  mRobot = std::make_shared<ConcreteRobot>(
      name,
      mRobotSkeleton,
      mSimulation,
      cloneRNG(),
      mTrajectoryExecutor,
      collisionDetector,
      selfCollisionFilter);

  // Set up HERB's AIKIDO planner. Right now, we only use the right arm.
  // TODO: (sniyaz) Include RRT in sequence.
  // TODO: (sniyaz) Enable bi-manual.
  auto rightArmStateSpace = mRightArm->getStateSpace();
  auto rightArmMetaSkeleton = mRightArm->getMetaSkeleton();

  auto snapConfigToConfigPlanner
      = std::make_shared<SnapConfigurationToConfigurationPlanner>(
          rightArmStateSpace,
          std::make_shared<GeodesicInterpolator>(rightArmStateSpace));

  auto dartSnapConfigToConfigPlanner = std::
      make_shared<ConfigurationToConfiguration_to_ConfigurationToConfiguration>(
          snapConfigToConfigPlanner, rightArmMetaSkeleton);

  auto snapConfigToTSRPlanner
      = std::make_shared<ConfigurationToConfiguration_to_ConfigurationToTSR>(
          snapConfigToConfigPlanner, rightArmMetaSkeleton);

  auto vfpOffsetPlanner
      = std::make_shared<VectorFieldConfigurationToEndEffectorOffsetPlanner>(
          rightArmStateSpace,
          rightArmMetaSkeleton,
          mPlannerParams.mVfpDistanceTolerance,
          mPlannerParams.mVfpPositionTolerance,
          mPlannerParams.mVfpAngularTolerance,
          mPlannerParams.mVfpInitialStepSize,
          mPlannerParams.mVfpJointLimitTolerance,
          mPlannerParams.mCollisionCheckResolution,
          mPlannerParams.mPlanningTimeout);

  // Create HERB's actual planner, a sequence meta-planner that contains each
  // of the above stand-alone planners.
  std::vector<std::shared_ptr<aikido::planner::Planner>> allPlanners = {
      dartSnapConfigToConfigPlanner, snapConfigToTSRPlanner, vfpOffsetPlanner};
  mPlanner
      = std::make_shared<SequenceMetaPlanner>(rightArmStateSpace, allPlanners);

  // Load the named configurations
  auto namedConfigurations = parseYAMLToNamedConfigurations(
      aikido::io::loadYAML(namedConfigurationsUri, retriever));
  mRobot->setNamedConfigurations(namedConfigurations);

  mThread = dart::common::make_unique<ExecutorThread>(
      std::bind(&Herb::update, this), mRosParams.mThreadExecutionUpdatePeriod);
}

//==============================================================================
std::future<void> Herb::executeTrajectory(const TrajectoryPtr& trajectory) const
{
  return mRobot->executeTrajectory(trajectory);
}

//==============================================================================
boost::optional<Eigen::VectorXd> Herb::getNamedConfiguration(
    const std::string& name) const
{
  return mRobot->getNamedConfiguration(name);
}

//==============================================================================
void Herb::setNamedConfigurations(
    std::unordered_map<std::string, const Eigen::VectorXd> namedConfigurations)
{
  mRobot->setNamedConfigurations(namedConfigurations);
}

//==============================================================================
std::string Herb::getName() const
{
  return mRobot->getName();
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr Herb::getMetaSkeleton() const
{
  return mRobot->getMetaSkeleton();
}

//==============================================================================
ConstMetaSkeletonStateSpacePtr Herb::getStateSpace() const
{
  return mRobot->getStateSpace();
}

//==============================================================================
void Herb::setRoot(Robot* robot)
{
  mRobot->setRoot(robot);
}

//==============================================================================
void Herb::step(const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mRobotSkeleton->getMutex());
  mRobot->step(timepoint);
  mLeftArm->step(timepoint);
  mRightArm->step(timepoint);
  mNeckPositionExecutor->step(timepoint);

  if (!mSimulation && mUpdatePositionFlag)
  {
    auto leftArmSkeleton = mLeftArm->getMetaSkeleton();
    auto rightArmSkeleton = mRightArm->getMetaSkeleton();

    leftArmSkeleton->setPositions(
        mJointStateClient->getLatestPosition(*leftArmSkeleton));
    rightArmSkeleton->setPositions(
        mJointStateClient->getLatestPosition(*rightArmSkeleton));
  }
}

//==============================================================================
CollisionFreePtr Herb::getSelfCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const
{
  return mRobot->getSelfCollisionConstraint(space, metaSkeleton);
}

//==============================================================================
TestablePtr Herb::getFullCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const CollisionFreePtr& collisionFree) const
{
  return mRobot->getFullCollisionConstraint(space, metaSkeleton, collisionFree);
}

//==============================================================================
std::unique_ptr<aikido::common::RNG> Herb::cloneRNG()
{
  return std::move(cloneRNGFrom(mRng)[0]);
}

//==============================================================================
aikido::planner::WorldPtr Herb::getWorld()
{
  return mWorld;
}

//==============================================================================
ConcreteManipulatorPtr Herb::getRightArm()
{
  return mRightArm;
}

//==============================================================================
ConcreteManipulatorPtr Herb::getLeftArm()
{
  return mLeftArm;
}

//==============================================================================
BarrettHandPtr Herb::getRightHand()
{
  return std::static_pointer_cast<BarrettHand>(mRightArm->getHand());
}

//==============================================================================
BarrettHandPtr Herb::getLeftHand()
{
  return std::static_pointer_cast<BarrettHand>(mLeftArm->getHand());
}

//==============================================================================
void Herb::update()
{
  step(std::chrono::system_clock::now());
}

//==============================================================================
TrajectoryPtr Herb::planToConfiguration(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const StateSpace::State* goalState,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  return mRobot->planToConfiguration(
      space, metaSkeleton, goalState, collisionFree, timelimit);
}

//==============================================================================
TrajectoryPtr Herb::planToConfiguration(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const Eigen::VectorXd& goal,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  return mRobot->planToConfiguration(
      space, metaSkeleton, goal, collisionFree, timelimit);
}

//==============================================================================
TrajectoryPtr Herb::planToConfigurations(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const std::vector<StateSpace::State*>& goalStates,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  return mRobot->planToConfigurations(
      space, metaSkeleton, goalStates, collisionFree, timelimit);
}

//==============================================================================
TrajectoryPtr Herb::planToConfigurations(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const std::vector<Eigen::VectorXd>& goals,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  return mRobot->planToConfigurations(
      space, metaSkeleton, goals, collisionFree, timelimit);
}

//==============================================================================
TrajectoryPtr Herb::planToTSR(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bn,
    const TSRPtr& tsr,
    const CollisionFreePtr& collisionFree,
    double timelimit,
    size_t maxNumTrials)
{
  return mRobot->planToTSR(
      space, metaSkeleton, bn, tsr, collisionFree, timelimit, maxNumTrials);
}

//==============================================================================
TrajectoryPtr Herb::planToNamedConfiguration(
    const std::string& name,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  return mRobot->planToNamedConfiguration(name, collisionFree, timelimit);
}

//==============================================================================
bool Herb::switchFromGravityCompensationControllersToTrajectoryExecutors()
{
  return switchControllers(trajectoryExecutors, gravityCompensationControllers);
}

//==============================================================================
bool Herb::switchFromTrajectoryExecutorsToGravityCompensationControllers()
{
  return switchControllers(gravityCompensationControllers, trajectoryExecutors);
}

//==============================================================================
std::future<void> Herb::setNeckPosition(const Eigen::VectorXd& position)
{
  return mNeckPositionExecutor->execute(position);
}

//==============================================================================
void Herb::say(const std::string& words)
{
  if (!mTalkerActionClient)
  {
    ROS_INFO_STREAM("Talker is not instantiated in simulation");
  }
  else
  {
    mTalkerActionClient->waitForServer();
    talker::SayGoal goal;
    goal.text = words;
    mTalkerActionClient->sendGoal(goal);
  }

  ROS_INFO_STREAM("HERB says: " << words);
}

//==============================================================================
void Herb::setVectorFieldPlannerParameters(
    const VectorFieldPlannerParameters& vfParameters)
{
  mLeftArm->setVectorFieldPlannerParameters(vfParameters);
  mRightArm->setVectorFieldPlannerParameters(vfParameters);
}

//==============================================================================
void Herb::setUpdatePositionsFromClient(bool flag)
{
  mUpdatePositionFlag = flag;
}

//==============================================================================
ConcreteManipulatorPtr Herb::configureArm(
    const std::string& armName,
    const dart::common::ResourceRetrieverPtr& retriever,
    dart::collision::CollisionDetectorPtr collisionDetector,
    const std::shared_ptr<dart::collision::BodyNodeCollisionFilter>&
        selfCollisionFilter)
{
  using dart::dynamics::Chain;

  std::stringstream wamBaseName;
  wamBaseName << "/" << armName << "/wam_base";

  std::stringstream armEndName;
  armEndName << "/" << armName << "/wam7";

  // Same as hand-base for HERB [offset from wam7 by FT sensor dimension]
  std::stringstream endEffectorName;
  endEffectorName << "/" << armName << "/hand_base";

  std::stringstream handBaseName;
  handBaseName << "/" << armName << "/hand_base";

  auto armBase = getBodyNodeOrThrow(mRobotSkeleton, wamBaseName.str());
  auto armEnd = getBodyNodeOrThrow(mRobotSkeleton, armEndName.str());

  auto arm = Chain::create(armBase, armEnd, armName + "_arm");
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(arm.get());

  auto hand = std::make_shared<BarrettHand>(
      armName,
      mSimulation,
      getBodyNodeOrThrow(mRobotSkeleton, endEffectorName.str()),
      getBodyNodeOrThrow(mRobotSkeleton, handBaseName.str()),
      selfCollisionFilter,
      mNode.get(),
      retriever);

  // Hardcoding to acceleration limits used in OpenRAVE
  // This is necessary because HERB is loaded from URDF, which
  // provides no means of specifying acceleration limits
  arm->setAccelerationLowerLimits(
      Eigen::VectorXd::Constant(arm->getNumDofs(), -2.0));
  arm->setAccelerationUpperLimits(
      Eigen::VectorXd::Constant(arm->getNumDofs(), 2.0));

  auto manipulatorRobot = std::make_shared<ConcreteRobot>(
      armName,
      arm,
      mSimulation,
      cloneRNG(),
      mTrajectoryExecutor,
      collisionDetector,
      selfCollisionFilter);

  auto manipulator
      = std::make_shared<ConcreteManipulator>(manipulatorRobot, hand);

  return manipulator;
}

//==============================================================================
std::shared_ptr<aikido::control::TrajectoryExecutor>
Herb::createTrajectoryExecutor()
{
  using aikido::control::KinematicSimulationTrajectoryExecutor;
  using aikido::control::ros::RosTrajectoryExecutor;

  if (mSimulation)
  {
    return std::make_shared<KinematicSimulationTrajectoryExecutor>(
        mRobotSkeleton);
  }
  else
  {
    // TODO: See https://github.com/personalrobotics/libherb/issues/96.
    std::string serverName
        = "bimanual_trajectory_controller/"
          "follow_joint_trajectory";
    return std::make_shared<RosTrajectoryExecutor>(
        *mNode,
        serverName,
        mRosParams.mTrajectoryInterpolationTimestep,
        mRosParams.mTrajectoryGoalTimeTolerance);
  }
}

//==============================================================================
dart::dynamics::ChainPtr Herb::configureNeck()
{
  using dart::dynamics::Chain;

  std::string neckName = "neck";
  std::string neckBaseName = "herb_frame";
  std::string neckEndName = "neck_tilt";

  auto neckBase = getBodyNodeOrThrow(mRobotSkeleton, neckBaseName);
  auto neckEnd = getBodyNodeOrThrow(mRobotSkeleton, neckEndName);

  auto neck = Chain::create(neckBase, neckEnd, neckName);
  return neck;
}

//==============================================================================
std::shared_ptr<aikido::control::PositionCommandExecutor>
Herb::createNeckPositionExecutor(const dart::dynamics::ChainPtr& neck)
{
  using aikido::control::ros::RosPositionCommandExecutor;

  if (mSimulation)
  {
    return std::
        make_shared<SchunkNeckKinematicSimulationPositionCommandExecutor>(neck);
  }
  else
  {
    const std::string serverName = "schunk_robot/position_controller/set_position";

    std::vector<std::string> jointNames{"pan", "tilt"};

    return std::make_shared<RosPositionCommandExecutor>(
        *mNode, serverName, jointNames);
  }
}

//==============================================================================
bool Herb::switchControllers(
    const std::vector<std::string>& start_controllers,
    const std::vector<std::string>& stop_controllers)
{
  if (!mNode)
    throw std::runtime_error("Ros node has not been instantiated.");

  if (!mControllerServiceClient)
    throw std::runtime_error("ServiceClient not instantiated.");

  controller_manager_msgs::SwitchController srv;
  srv.request.start_controllers = start_controllers;
  srv.request.stop_controllers = stop_controllers;
  srv.request.strictness
      = controller_manager_msgs::SwitchControllerRequest::STRICT;

  if (mControllerServiceClient->call(srv))
    return srv.response.ok;
  else
    throw std::runtime_error("SwitchController failed.");
}

} // ns
