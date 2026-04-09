#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/LookAtTask.h>
#include "api.h"

struct JVRC1Controller_DLLAPI JVRC1Controller : public mc_control::MCController
{
  JVRC1Controller(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;

  private:
    // State machine for the whole sequence of movements
    enum class State
    {
      leftHandForward, //move left hand to target position
      leftHandBack, //move left hand back to initial position
      rightHandForward,  //move right hand to target position
      rightHandBack, //move right hand back to initial position
      bothHandsForward, //move both hands to target position at the same time
      bothHandsBack //move both hands back to initial position at the same time
    };

    // Tolerance configuration for task completion

    /*---------------Justification for different tolerances----------------
    I decided to use different tolerances for forward and backward movements to account for potential differences
    in task difficulty, I saw the solver was having some issues to keep the posture and balance so I adjusted the tolerances.
    The "both" tolerance is used when both hands are moving simultaneously, which may require a slightly looser tolerance due
    to the increased complexity of coordinating both arms. The "lookAt" tolerance is just to ensure that the robot's gaze is 
    sufficiently close to the target before considering the task complete.*/

    struct toleranceConfig
    {
      double forward; //tolerance to reach the target
      double back; //tolerance to go back to the initial position
      double both; //tolerance when both hands are moving at the same time
      double lookAt; //tolerance for the lookAt task
    };


    mc_rtc::Configuration config_; // Store the configuration for later use
    double timeStep; // Store the time step for use in timing conditions
    double minStateTime; // Store the minimum time spent in each state to avoid state flickering due to operational noise
    double maxStateTime; // Store the maximum time spent in each state to ensure the robot doesn't get stuck in a state in case of issues

    toleranceConfig tol_conf; // Store the tolerance values which are loaded from the yaml configuration file 
    State state = State::leftHandForward; // Initial state of the FSM
    double stateTime = 0.0; // Time spent in the current state, used for timing conditions

    Eigen::Vector3d lookForwardTarget; // Target point for the LookAtTask when looking forward
    sva::PTransformd lhand_target; // Target pose for the left hand
    sva::PTransformd rhand_target; // Target pose for the right hand
    sva::PTransformd lhand_init; // Initial pose of the left hand, used for moving back to the original position
    sva::PTransformd rhand_init; // Initial pose of the right hand, used for moving back to the original position
    
    std::shared_ptr<mc_tasks::EndEffectorTask> leftHandTask; // Task for controlling the left hand
    std::shared_ptr<mc_tasks::EndEffectorTask> rightHandTask; // Task for controlling the right hand
    std::shared_ptr<mc_tasks::LookAtTask> lookAtTask; // Task for controlling the robot's gaze to look at a target point
    std::shared_ptr<mc_tasks::CoMTask> comTask; // Task for controlling the robot's center of mass

    void runLeftHandForward(); // Function to execute the movement of the left hand towards the target position
    void runLeftHandBack(); // Function to execute the movement of the left hand back to the initial position
    void runRightHandForward(); // Function to execute the movement of the right hand towards the target position
    void runRightHandBack(); // Function to execute the movement of the right hand back to the initial position
    void runBothHandsForward(); // Function to execute the movement of both hands towards their respective target positions 
    void runBothHandsBack(); // Function to execute the movement of both hands back to their initial positions
    void lookAtRight(); // Function to look at the right hand
    void lookAtLeft(); // Function to look at the left hand
    void lookForward(); // Function to look forward 
    bool checkTransition(bool successCondition, State nextState, const std::string & successMsg); // Function to check if the conditions for transitioning to the next state are met
    void changeState(State new_state); // Function to change the state of the FSM
    
};