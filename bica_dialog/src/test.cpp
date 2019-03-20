#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sound_play/SoundRequestAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_sound");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<sound_play::SoundRequestAction> ac("sound_play", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  sound_play::SoundRequestGoal goal;
  goal.sound_request.sound = sound_play::SoundRequest::SAY;
  goal.sound_request.command = sound_play::SoundRequest::PLAY_ONCE;
  goal.sound_request.arg = "Testing the actionlib interface A P I";
  goal.sound_request.volume = 1.0;
  ac.sendGoal(goal);


  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
