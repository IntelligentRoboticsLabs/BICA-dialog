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
#ifndef DIALOGINTERFACE__H
#define DIALOGINTERFACE__H

#include <ros/ros.h>
#include <string>
#include <dialogflow_ros_msgs/DialogflowResult.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sound_play/SoundRequestAction.h>
#include <regex>

namespace bica_dialog
{
class DialogInterface
{
public:
  DialogInterface(std::string intent);
  DialogInterface(std::regex intent_re);
  DialogInterface();

  void dfCallback(const dialogflow_ros_msgs::DialogflowResult::ConstPtr& result);
  bool speak(std::string str);
  bool listen();
  virtual void listenCallback(dialogflow_ros_msgs::DialogflowResult result){}
  std::string getIntent();
  std::regex getIntentRegex();

protected:
  ros::NodeHandle nh_;
  std::string intent_, results_topic_, start_srv_, speak_topic_;
  bool is_bussy_;
  ros::ServiceClient df_srv_;
  ros::Subscriber df_result_sub_;
  ros::Publisher speak_pub_;
  actionlib::SimpleActionClient<sound_play::SoundRequestAction> ac;
  std::regex intent_re_;
  void init();
};
};  // namespace bica_dialog

#endif
