/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Jonatan Gines jginesclavero@gmail.com */

/* Mantainer: Jonatan Gines jginesclavero@gmail.com */
#include <bica_dialog/DialogInterface.h>

#include <string>

namespace bica_dialog
{
DialogInterface::DialogInterface(std::string intent) :
  intent_(intent), nh_(), is_bussy_(false), ac("sound_play", true)
{
  init();
}

DialogInterface::DialogInterface(std::regex intent_re) :
  intent_re_(intent_re), nh_(), is_bussy_(false), ac("sound_play", true)
{
  init();
}

DialogInterface::DialogInterface() : nh_(), is_bussy_(false), ac("sound_play", true)
{
  init();
}

void DialogInterface::init()
{
  if (!nh_.getParam("/dialogflow_client/results_topic", results_topic_))
    results_topic_ = "/dialogflow_client/results";
  if (!nh_.getParam("/dialogflow_client/start_srv", start_srv_))
    start_srv_ = "/dialogflow_client/start";
  speak_topic_ = "/bica_dialog/speak";
  df_result_sub_ = nh_.subscribe(results_topic_, 1, &DialogInterface::dfCallback, this);
}

std::string DialogInterface::getIntent()
{
  return intent_;
}

std::regex DialogInterface::getIntentRegex()
{
  return intent_re_;
}

void DialogInterface::dfCallback(const dialogflow_ros::DialogflowResult::ConstPtr& result)
{
  if(result->intent == intent_ || std::regex_match(result->intent, intent_re_))
  {
    is_bussy_ = false;
    listenCallback(*result);
  }
}

bool DialogInterface::speak(std::string str)
{
  if (is_bussy_)
    return !is_bussy_;
  else
  {
    ac.waitForServer();
    sound_play::SoundRequestGoal goal;
    goal.sound_request.sound = sound_play::SoundRequest::SAY;
    goal.sound_request.command = sound_play::SoundRequest::PLAY_ONCE;
    goal.sound_request.arg = str;
    goal.sound_request.volume = 1.0;
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Sound_play Action finished: %s",state.toString().c_str());
      return true;
    }
    else
    {
      ROS_INFO("Sound_play Action did not finish before the time out.");
      return false;
    }
  }
}

bool DialogInterface::listen()
{
  std_srvs::Empty srv;
  ros::ServiceClient df_srv = nh_.serviceClient<std_srvs::Empty>(start_srv_, 1);
  df_srv.call(srv);
  return true;
}

};  // namespace bica_dialog
