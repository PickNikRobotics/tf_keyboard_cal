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
#include <QGroupBox>

#include "tf_keyboard_cal_gui.h"

namespace tf_keyboard_cal
{
TFKeyboardCalGui::TFKeyboardCalGui(QWidget* parent) : rviz::Panel(parent)
{
  tabWidget_ = new QTabWidget;
  tabWidget_->addTab(new createTFTab(), tr("Add / Remove"));
  tabWidget_->addTab(new manipulateTFTab(), tr("Manipulate"));
  tabWidget_->addTab(new saveLoadTFTab(), tr("Save / Load"));

  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addWidget(tabWidget_);
  setLayout(main_layout);  
}

createTFTab::createTFTab(QWidget *parent) : QWidget(parent)
{

  // TF controls
  QLabel *from_label = new QLabel(tr("from:"));
  QLabel *to_label = new QLabel(tr("to:"));
  
  from_ = new QLineEdit;
  from_->setPlaceholderText("from TF");
  connect(from_, SIGNAL(textChanged(const QString &)), this, SLOT(fromTextChanged(const QString &)));
  
  to_ = new QLineEdit;
  to_->setPlaceholderText("to TF");
  connect(to_, SIGNAL(textChanged(const QString &)), this, SLOT(toTextChanged(const QString &)));

  create_tf_btn_ = new QPushButton(this);
  create_tf_btn_->setText("Create TF");
  connect(create_tf_btn_, SIGNAL(clicked()), this, SLOT(createNewTF()));

  remove_tf_btn_ = new QPushButton(this);
  remove_tf_btn_->setText("Remove TF");
  connect(remove_tf_btn_, SIGNAL(clicked()), this, SLOT(removeTF()));

  active_tfs_ = new QComboBox;
  active_tfs_->addItem(tr("Select TF to delete"));

  // Layout
  QGroupBox *add_section = new QGroupBox(tr("Add TF"));

  QHBoxLayout *from_row = new QHBoxLayout;
  from_row->addWidget(from_label);
  from_row->addWidget(from_);

  QHBoxLayout *to_row = new QHBoxLayout;
  to_row->addWidget(to_label);
  to_row->addWidget(to_);

  QHBoxLayout *remove_row = new QHBoxLayout;
  remove_row->addWidget(active_tfs_);
  remove_row->addWidget(remove_tf_btn_);

  QVBoxLayout *add_controls = new QVBoxLayout;
  add_controls->addLayout(from_row);
  add_controls->addLayout(to_row);
  add_controls->addWidget(create_tf_btn_);
  add_section->setLayout(add_controls);

  QGroupBox *remove_section = new QGroupBox(tr("Remove TF"));
  
  QVBoxLayout *remove_controls = new QVBoxLayout;
  remove_controls->addLayout(remove_row); 
  remove_section->setLayout(remove_controls);
  
  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addWidget(add_section);
  main_layout->addWidget(remove_section);
  setLayout(main_layout);
}

void createTFTab::createNewTF()
{
  ROS_DEBUG_STREAM_NAMED("createNewTF","create new TF button pressed.");
  ROS_DEBUG_STREAM_NAMED("createNewTF","from:to = " << from_tf_name_ << ":" << to_tf_name_);
}

void createTFTab::removeTF()
{
  std::string tf_id = active_tfs_->currentText().toStdString();
  ROS_DEBUG_STREAM_NAMED("removeTF","remove TF: " << active_tfs_->currentIndex() << ", " << tf_id);
}

void createTFTab::fromTextChanged(QString text)
{
  from_tf_name_ = text.toStdString();
  ROS_DEBUG_STREAM_NAMED("fromTextChanged","from: " << from_tf_name_); 
}

void createTFTab::toTextChanged(QString text)
{
  to_tf_name_ = text.toStdString();
  ROS_DEBUG_STREAM_NAMED("toTextChanged","to: " << to_tf_name_);
}

manipulateTFTab::manipulateTFTab(QWidget *parent) : QWidget(parent)
{
  QLabel *some_text = new QLabel(tr("Some Text"));

  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addWidget(some_text);
  setLayout(main_layout);
}

saveLoadTFTab::saveLoadTFTab(QWidget *parent) : QWidget(parent)
{
  QLabel *some_text = new QLabel(tr("Some Text"));

  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addWidget(some_text);
  setLayout(main_layout);
}

} // end namespace tf_keyboard_cal

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tf_keyboard_cal::TFKeyboardCalGui, rviz::Panel)
