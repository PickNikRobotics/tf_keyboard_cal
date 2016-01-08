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
 * Author : Andy McEvoy, Dave Coleman
 * Desc   : Allows manual control of a TF through the keyboard
 */

#include <tf_keyboard_cal/manual_tf_alignment.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <iostream>
#include <fstream>
#include <string>

#include <ros/package.h>

namespace tf_keyboard_cal
{

ManualTFAlignment::ManualTFAlignment()
  : nh_("~")
  , name_("manipulation_data") // for namespacing logging messages
{
  // set defaults
  mode_ = 1;
  delta_ = 0.010;

  // initial camera transform
  double x, y, z, roll, pitch, yaw;

  // Get settings from rosparam
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh_, "initial_x", x);
  error += !rosparam_shortcuts::get(name_, nh_, "initial_y", y);
  error += !rosparam_shortcuts::get(name_, nh_, "initial_z", z);
  error += !rosparam_shortcuts::get(name_, nh_, "initial_roll", roll);
  error += !rosparam_shortcuts::get(name_, nh_, "initial_pitch", pitch);
  error += !rosparam_shortcuts::get(name_, nh_, "initial_yaw", yaw);
  error += !rosparam_shortcuts::get(name_, nh_, "file_package", file_package_);
  error += !rosparam_shortcuts::get(name_, nh_, "file_name", file_name_);
  error += !rosparam_shortcuts::get(name_, nh_, "topic_name", topic_name_);
  error += !rosparam_shortcuts::get(name_, nh_, "from", from_);
  error += !rosparam_shortcuts::get(name_, nh_, "to", to_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  setPose(Eigen::Vector3d(x, y, z), Eigen::Vector3d(roll, pitch, yaw));

  // default, save in tf_keyboard_cal/data
  std::string package_path = ros::package::getPath(file_package_);
  save_path_ = package_path + file_name_;

  // listen to keyboard topic
  keyboard_sub_ = nh_.subscribe(topic_name_, 100,
                                &ManualTFAlignment::keyboardCallback, this);

  // Echo info
  ROS_INFO_STREAM_NAMED("manualTF","Listening to topic : " << topic_name_);
  ROS_INFO_STREAM_NAMED("manualTF","Transform from     : " << from_);
  ROS_INFO_STREAM_NAMED("manualTF","Transform to       : " << to_);
  ROS_INFO_STREAM_NAMED("manualTF","Config File        : " << save_path_);
  ROS_INFO_STREAM_NAMED("manualTF","Initial transform  : " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw );
}

void ManualTFAlignment::keyboardCallback(const keyboard::Key::ConstPtr& msg)
{
  int entry = msg->code;
  const double fine = 0.001;
  const double coarse = 0.01;
  const double very_coarse = 0.1;

  //std::cout << "key: " << entry << std::endl;

  switch(entry)
  {
    case 112: //
      writeTFToFile();
      break;
    case 117: // (very coarse delta)
      std::cout << "Delta = very coarse (0.1)" << std::endl;
      delta_ = very_coarse;
      break;
    case 105: // (coarse delta)
      std::cout << "Delta = coarse (0.01)" << std::endl;
      delta_ = coarse;
      break;
    case 111: // (fine delta)
      std::cout << "Delta = fine (0.001)" << std::endl;
      delta_ = fine;
      break;

    // X axis
    case 113: // up
      updateTF(1, delta_);
      break;
    case 97: // down
      updateTF(1, -delta_);
      break;

    // y axis
    case 119: // up
      updateTF(2, delta_);
      break;
    case 115: // down
      updateTF(2, -delta_);
      break;

    // z axis
    case 101: // up
      updateTF(3, delta_);
      break;
    case 100: // down
      updateTF(3, -delta_);
      break;

    // roll
    case 114: // up
      updateTF(4, delta_);
      break;
    case 102: // down
      updateTF(4, -delta_);
      break;

    // pitch
    case 116: // up
      updateTF(5, delta_);
      break;
    case 103: // down
      updateTF(5, -delta_);
      break;

    // yaw
    case 121: // up
      updateTF(6, delta_);
      break;
    case 104: // down
      updateTF(6, -delta_);
      break;

    default:
      // don't do anything
      break;
  }
}

void ManualTFAlignment::printMenu()
{
  std::cout << "Manual alignment of camera to world CS:" << std::endl;
  std::cout << "=======================================" << std::endl;
  std::cout << "MOVE: X  Y  Z  R  P  YAW " << std::endl;
  std::cout << "------------------------" << std::endl;
  std::cout << "up    q  w  e  r  t  y " << std::endl;
  std::cout << "down  a  s  d  f  g  h " << std::endl;
  std::cout << std::endl;
  std::cout << "Fast: u " << std::endl;
  std::cout << "Med:  i " << std::endl;
  std::cout << "Slow: o " << std::endl;
  std::cout << "Save: p " << std::endl;
}

void ManualTFAlignment::publishTF()
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  // set camera pose translation
  transform.setOrigin( tf::Vector3( translation_[0],
                                    translation_[1],
                                    translation_[2]) );

  // set camera pose rotation
  q.setRPY(rotation_[0], rotation_[1], rotation_[2]);
  transform.setRotation(q);

  // publish
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), from_ , to_));
}

void ManualTFAlignment::setPose(Eigen::Vector3d translation, Eigen::Vector3d rotation)
{
  translation_ = translation;
  rotation_ = rotation;
}

void ManualTFAlignment::updateTF(int mode, double delta)
{
  ROS_DEBUG_STREAM_NAMED("tf_alignment","mode = " << mode << ", delta = " << delta);

  switch(mode)
  {
    case 1:
      translation_ += Eigen::Vector3d(delta, 0, 0);
      break;
    case 2:
      translation_ += Eigen::Vector3d(0, delta, 0);
      break;
    case 3:
      translation_ += Eigen::Vector3d(0, 0, delta);
      break;
    case 4:
      rotation_ += Eigen::Vector3d(delta, 0, 0);
      break;
    case 5:
      rotation_ += Eigen::Vector3d(0, delta, 0);
      break;
    case 6:
      rotation_ += Eigen::Vector3d(0, 0, delta);
      break;
    default:
      // don't do anything
      break;
  }
}


void ManualTFAlignment::writeTFToFile()
{
  std::ofstream file (save_path_.c_str()); //, std::ios::app);
  ROS_INFO_STREAM_NAMED("tf_align.write","Writing transformation to file " << save_path_);

  if (!file.is_open())
    ROS_ERROR_STREAM_NAMED("tf_align.write","Output file could not be opened: " << save_path_);
  else
  {
    ROS_INFO_STREAM_NAMED("tf_align.write","Initial transform  : " << translation_[0] << ", " << translation_[1]
                          << ", " << translation_[2] << ", " << rotation_[0] << ", " << rotation_[1]
                          << ", " << rotation_[2] );

    file << "initial_x: " << translation_[0] << std::endl;
    file << "initial_y: " << translation_[1] << std::endl;
    file << "initial_z: " << translation_[2] << std::endl;
    file << "initial_roll: " << rotation_[0] << std::endl;
    file << "initial_pitch: " << rotation_[1] << std::endl;
    file << "initial_yaw: " << rotation_[2] << std::endl;
    file << "from: " << from_ << std::endl;
    file << "to: " << to_ << std::endl;
    file << "file_package: " << file_package_ << std::endl;
    file << "file_name: " << file_name_ << std::endl;
    file << "topic_name: " << topic_name_ << std::endl;
  }
  file.close();

}

}
