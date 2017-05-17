/****************************************************************************************************
 *  Software License Agreement (BSD License)
 *  
 *  Copyright 2017, Andy McEvoy
 *  
 *  Redistribution and use in source and binary forms, with or without modification, are permitted
 *  provided that the following conditions are met:
 *  
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *  and the following disclaimer.
 *  
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of
 *  conditions and the following disclaimer in the documentation and/or other materials provided
 *  with the distribution.
 *  
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to
 *  endorse or promote products derived from this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 *  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 *  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************************************/
/*
 * Author(s) : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc      : implementation of TFRemoteReceiver. See H file for documentation.
 * Created   : 09 - May - 2017
 */

#include <tf_keyboard_cal/tf_remote_receiver.h>

namespace tf_keyboard_cal
{

TFRemoteReceiver::TFRemoteReceiver()
  : nh_("~")
{
  std::cout << "\033[1;36m" << "remote receiver initialized" << "\033[0m" << std::endl;

  create_tf_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/create_rviz_tf", 10);
  remove_tf_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/remove_rviz_tf", 10);
  tf_sub_ = nh_.subscribe("/tf", 1, &TFRemoteReceiver::tf_callback, this);
}

void TFRemoteReceiver::tf_callback(const tf2_msgs::TFMessage &msg)
{
  // get names of published tfs
}

void TFRemoteReceiver::createTF(geometry_msgs::TransformStamped new_tf_msg)
{
  create_tf_pub_.publish(new_tf_msg);
}

void TFRemoteReceiver::removeTF()
{
  std::cout << "\033[1;36m" << "remove tf" << "\033[0m" << std::endl;
}

void TFRemoteReceiver::updateTF()
{
  std::cout << "\033[1;36m" << "update tf" << "\033[0m" << std::endl;
}

} // end namespace tf_keyboard_cal
