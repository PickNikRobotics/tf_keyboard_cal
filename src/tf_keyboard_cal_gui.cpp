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
 * Desc      : Rviz display panel for dynamically manipulating TFs
 * Created   : 26 - April - 2017
 */
#include <stdio.h>


#include <QHBoxLayout>
#include <QVBoxLayout>

#include "tf_keyboard_cal_gui.h"


namespace tf_keyboard_cal
{
TFKeyboardCalGui::TFKeyboardCalGui(QWidget* parent) : rviz::Panel(parent)
{
  std::cout << "\033[1;36m" << "TFKeyboardCalGui" << "\033[0m" << std::endl;
  // create a test button
  btn_test_ = new QPushButton(this);
  btn_test_->setText("Test");
  connect(btn_test_, SIGNAL(clicked()), this, SLOT(testFnc()));

  // Vertical Layout
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(btn_test_);
  setLayout(layout);
  std::cout << "\033[1;36m" << "end" << "\033[0m" << std::endl;
}

void TFKeyboardCalGui::testFnc()
{
  
}

void TFKeyboardCalGui::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void TFKeyboardCalGui::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

} // end namespace tf_keyboard_cal

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tf_keyboard_cal::TFKeyboardCalGui, rviz::Panel)
