/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author : Andy McEvoy (mcevoy.andy@gmail.com), Dave Coleman
 * Desc   : Allows manual control of a TF through the keyboard
 */

#ifndef TF_KEYBOARD_CAL__MANUAL_TF_ALIGNMENT_
#define TF_KEYBOARD_CAL__MANUAL_TF_ALIGNMENT_

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <keyboard/Key.h>

#include <Eigen/Core>

#include <boost/filesystem.hpp>

namespace tf_keyboard_cal
{

class ManualTFAlignment
{
public:

  /*
   * \brief
   */
  ManualTFAlignment();
    
  /*
   * \brief
   */
  void keyboardCallback(const keyboard::Key::ConstPtr& msg);
  
  /*
   * \brief
   */
  void printMenu();

  /* 
   * \brief 
   */
  void publishTF();

  /*
   * \brief
   */
  void setPose(Eigen::Vector3d translation, Eigen::Vector3d rotation);

  /*
   * \brief
   */
  void updateTF(int mode, double delta);
  
  /*
   * \brief
   */
  void writeTFToFile();

  Eigen::Vector3d translation_;
  Eigen::Vector3d rotation_;
  std::string save_path_;
  int mode_;
  double delta_;
  std::string from_;
  std::string to_;
  std::string file_package_;
  std::string file_name_;
  std::string topic_name_;
  ros::Subscriber keyboard_sub_;
  ros::NodeHandle nh_;

};

} // end namespace

#endif
