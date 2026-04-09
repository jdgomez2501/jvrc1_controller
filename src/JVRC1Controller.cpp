#include "JVRC1Controller.h"

JVRC1Controller::JVRC1Controller(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  config_.load(config); //loading the yaml config to be used later
  timeStep = dt; // Getting the time step used by the controller
  minStateTime = config_("timing")("minStateTime"); // Getting the minimum time spent in each state from the config file
  maxStateTime = config_("timing")("maxStateTime"); // Getting the maximum time spent in each state from the config file

  solver().addConstraintSet(contactConstraint); // enforcing contact constraints
  solver().addConstraintSet(dynamicsConstraint); // enforcing dynamics constraints (equation of motion)
  solver().addConstraintSet(kinematicsConstraint); // enforcing joint constraints
  solver().addConstraintSet(selfCollisionConstraint); // enforcing self-collision avoidance

  // enforcing left and right foot contacts with the ground
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});

  // ----------- POSTURE AND COM TASKS -----------
  // Posture task to maintain a natural posture and help with balance during the movements
  solver().addTask(postureTask);
  const auto post_conf = config_("posture");
  postureTask->stiffness(post_conf("stiffness"));
  postureTask->weight(post_conf("weight"));

  // CoM task to control the center of mass, which is crucial for maintaining balance during the movements
  const auto com_conf = config_("com");

  /* I decided to modify the dim_weight of the CoM task to give some freedom in the motion, 
  which can help the solver find solutions that maintain balance while moving, 
  especially during the hand movements which can cause shifts in the robot's balance. I tunned these values
  by experimentation and then defined them in the configuration file */

  const Eigen::Vector3d com_dim = 
    Eigen::Vector3d(
      com_conf("dim_weight")[0],
      com_conf("dim_weight")[1],
      com_conf("dim_weight")[2]);

  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, com_conf("stiffness"), com_conf("weight"));
  solver().addTask(comTask);
  comTask->dimWeight(com_dim);

  //----------- HAND TASKS------------
  const auto hands_target = config_("handsTarget");

  // LEFT HAND
  const auto l_pos_conf = hands_target("lhand_position");
  const auto l_quat_conf = hands_target("lhand_quaternion");

  const Eigen::Vector3d l_pos(
    l_pos_conf[0],
    l_pos_conf[1],
    l_pos_conf[2]);
  const Eigen::Quaterniond l_quat(
    l_quat_conf[0], // w
    l_quat_conf[1], // x
    l_quat_conf[2], // y
    l_quat_conf[3]);  // z

  lhand_target = sva::PTransformd(
    l_quat.toRotationMatrix(),
    l_pos);

  // RIGHT HAND
  const auto r_pos_conf = hands_target("rhand_position");
  const auto r_quat_conf = hands_target("rhand_quaternion");

  const Eigen::Vector3d r_pos(
    r_pos_conf[0],
    r_pos_conf[1],
    r_pos_conf[2]);

  const Eigen::Quaterniond r_quat(
    r_quat_conf[0],
    r_quat_conf[1],
    r_quat_conf[2],
    r_quat_conf[3]);

  rhand_target = sva::PTransformd(
      r_quat.toRotationMatrix(),
      r_pos
  );

  const auto hands_conf = config_("hands");

  leftHandTask = std::make_shared<mc_tasks::EndEffectorTask>("l_wrist", robots(), 0, hands_conf("stiffness"), hands_conf("weight"));
  solver().addTask(leftHandTask);

  rightHandTask = std::make_shared<mc_tasks::EndEffectorTask>("r_wrist", robots(), 0, hands_conf("stiffness"), hands_conf("weight"));
  solver().addTask(rightHandTask);

  // ----------- LOOKAT TASK -----------
  const auto lookAt_conf = config_("lookAt");
  
  /* I am using depth camera frame for the lookat task since it is located 
  between the eyes and gives a better representation of the robot's gaze direction */
  const std::string cameraFrame = "dcamera";

  // Define the target when looking forward, A point far away in front of the robot at the same height as the depth camera
  lookForwardTarget = Eigen::Vector3d(100.0, 0.0, robots().robot(0).frame(cameraFrame).position().translation().z());

  // Create LookAtTask
  lookAtTask = std::make_shared<mc_tasks::LookAtTask>(
      cameraFrame,                   // body name to rotate (head/camera)
      Eigen::Vector3d::UnitZ(),   // forward vector of body
      lookForwardTarget,           // initial target position
      robots(),                    // robots
      0,                           // robot index
      lookAt_conf("stiffness"),
      lookAt_conf("weight")
  );

  solver().addTask(lookAtTask);

  //----------- TOLERANCES -----------
  const auto tolerance_conf = config_("tolerance");
  tol_conf.forward = tolerance_conf("forward");
  tol_conf.back = tolerance_conf("back");
  tol_conf.both = tolerance_conf("both");
  tol_conf.lookAt = tolerance_conf("lookAt");

  mc_rtc::log::success("JVRC1Controller init done ");

}

bool JVRC1Controller::run()
{
  // Here we only have the logic for the FSM that controls the sequence of movements.

  stateTime += timeStep; 

  switch(state)
  {
    case State::leftHandForward:
      runLeftHandForward();
      break;

    case State::leftHandBack:
      runLeftHandBack();
      break;

    case State::rightHandForward:
      runRightHandForward();
      break;

    case State::rightHandBack:
      runRightHandBack();
      break;

    case State::bothHandsForward:
      runBothHandsForward();
      break;

    case State::bothHandsBack:
      runBothHandsBack();
      break;

    default:
      mc_rtc::log::error("Unknown FSM state!");
      break;
  }
  return mc_control::MCController::run();

}

void JVRC1Controller::reset(const mc_control::ControllerResetData & reset_data)
{

  //Here we ensure correct reset and inisialization of all tasks.
  mc_control::MCController::reset(reset_data);
  comTask->reset();
  comTask->com(robot().com());
  leftHandTask->reset();
  rightHandTask->reset();
  lookAtTask->reset();

  // Store the initial poses of the hands to be used for moving back to the original position
  lhand_init = sva::PTransformd{leftHandTask->get_ef_pose().rotation(), leftHandTask->get_ef_pose().translation()};
  rhand_init = sva::PTransformd{rightHandTask->get_ef_pose().rotation(), rightHandTask->get_ef_pose().translation()};

  // Reset the FSM to the initial state
  state = State::leftHandForward;
  
}

void JVRC1Controller::runLeftHandForward()
{
  // Move left hand to target position
  leftHandTask->set_ef_pose(lhand_target);

  // Look at left hand
  lookAtLeft();

  // Check if the tasks are completed within the specified tolerances to transition to the next state
  bool success =  leftHandTask->eval().norm() < tol_conf.forward;
  checkTransition(success, State::leftHandBack,"Target reached, left hand going back to original position");

}

void JVRC1Controller::runLeftHandBack()
{
  // Move left hand back to initial position
  leftHandTask->set_ef_pose(lhand_init);

  // Look at left hand
  lookAtLeft();
  bool success =  leftHandTask->eval().norm() < tol_conf.back;
  checkTransition(success, State::rightHandForward,"Target reached, right hand going to desired position");
}

void JVRC1Controller::runRightHandForward()
{
  // Move right hand to target position
  rightHandTask->set_ef_pose(rhand_target);

  // Look at right hand
  lookAtRight();

  bool success =  rightHandTask->eval().norm() < tol_conf.forward;
  checkTransition(success, State::rightHandBack,"Target reached, right hand going back to original position");
}

void JVRC1Controller::runRightHandBack()
{
  // Move right hand back to initial position
  rightHandTask->set_ef_pose(rhand_init);

  // Look at right hand
  lookAtRight();
  bool success =  rightHandTask->eval().norm() < tol_conf.back;
  checkTransition(success, State::bothHandsForward,"Target reached, both hands going to desired position");
}

void JVRC1Controller::runBothHandsForward()
{
  // Move both hands to target positions
  rightHandTask->set_ef_pose(rhand_target);
  leftHandTask->set_ef_pose(lhand_target);

  // Look forward
  lookForward();
  bool success =  rightHandTask->eval().norm() < tol_conf.both && leftHandTask->eval().norm() < tol_conf.both;
  checkTransition(success, State::bothHandsBack,"Target reached, both hands going back to original position");

}

void JVRC1Controller::runBothHandsBack()
{
  // Move both hands back to initial positions
  rightHandTask->set_ef_pose(rhand_init);
  leftHandTask->set_ef_pose(lhand_init);

  // Look forward
  lookForward();

  bool success =  rightHandTask->eval().norm() < tol_conf.back && leftHandTask->eval().norm() < tol_conf.back;
  checkTransition(success, State::leftHandForward,"Target reached, restarting the sequence with left hand going to desired position");

}

void JVRC1Controller::lookAtLeft()
{
    lookAtTask->target(robot().frame("l_wrist").position().translation());
}

void JVRC1Controller::lookAtRight()
{
    lookAtTask->target(robot().frame("r_wrist").position().translation());
}

void JVRC1Controller::lookForward()
{
    lookAtTask->target(lookForwardTarget);
}

bool JVRC1Controller::checkTransition(bool successCondition, State nextState, const std::string & successMsg)
{
  // Check if the 3 main coinditions are met to transition to the next state: 
  // 1) The hand(s) task(s) for the current state are completed within the specified tolerances
  // 2) The robot has been in the current state for at least the minimum time to avoid state flickering due to operational noise
  // 3) The robot is looking at the target

  if(successCondition && lookAtTask->eval().norm() < tol_conf.lookAt && stateTime > minStateTime)
  {
    mc_rtc::log::success(successMsg);
    changeState(nextState);
    return true;
  }

  // If conditions are not met we put a timeout to avoid being stuck in a state forever due to unforeseen issues or 
  // unachievable targets. This ensures the robot can keep trying to execute the sequence even if something goes wrong.
  // For example, if we change the tolerance values to be too strict the robot might not be able to achieve the targets
  // so after the timeout it will force the transition to aovid getting stuck in the current state. 
  if(stateTime > maxStateTime)
  {
    mc_rtc::log::warning("State timeout reached, forcing transition to next state");
    changeState(nextState);
    return true;
  }

  return false;
}

void JVRC1Controller::changeState(State newState)
{
  state = newState;
  stateTime = 0.0;
}


CONTROLLER_CONSTRUCTOR("JVRC1Controller", JVRC1Controller)
