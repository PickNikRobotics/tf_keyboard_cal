/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
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

/* Author: Dave Coleman
   Desc:
*/

#ifndef TF_KEYBOARD_CAL_DEMO_TF_LISTENER_H
#define TF_KEYBOARD_CAL_DEMO_TF_LISTENER_H

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// Visualize
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace tf_keyboard_cal
{
class DemoTFListener
{
public:
  /**
   * \brief Constructor
   */
  DemoTFListener() : name_("demo_tf_listener"), nh_("~")
  {
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/demo_tf_listener/markers"));
    visual_tools_->deleteAllMarkers();

    ROS_INFO_STREAM_NAMED(name_, "DemoTFListener Ready.");
  }

  void runLoop()
  {
    // Allow time for listener to load
    ros::Duration(1.0).sleep();

    const bool verbose = false;

    ros::Rate rate(10.0);
    while (ros::ok())
    {
      Eigen::Affine3d pose;

      getPose("/world", "/thing", pose);

      // Show pose in console
      if (verbose)
        std::cout << "Position: \n" << pose.translation() << std::endl;

      // Visualize pose
      visual_tools_->publishXArrow(pose);

      rate.sleep();
    }
  }

  bool getPose(const std::string& from_frame, const std::string& to_frame, Eigen::Affine3d &pose)
  {
      tf::StampedTransform tf_transform;
      try
      {
        tf_.lookupTransform(from_frame, to_frame, ros::Time(0), tf_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        return false;
      }

      // Convert to eigen
      tf::transformTFToEigen(tf_transform, pose);
      return true;
  }

private:
  // --------------------------------------------------------
  // The short name of this class
  std::string name_;

  // A shared node handle
  ros::NodeHandle nh_;

  // For visualizing things in rviz
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  tf::TransformListener tf_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<DemoTFListener> DemoTFListenerPtr;
typedef boost::shared_ptr<const DemoTFListener> DemoTFListenerConstPtr;

}  // namespace tf_keyboard_cal
#endif  // TF_KEYBOARD_CAL_DEMO_TF_LISTENER_H

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "demo_tf_listener");
  ROS_INFO_STREAM_NAMED("main", "Starting DemoTFListener...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Initialize main class
  tf_keyboard_cal::DemoTFListener server;
  server.runLoop();

  // Shutdown
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
