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
#include <QFileDialog>
#include <QGridLayout>

#include <tf/transform_datatypes.h>

// TODO: Why doesn't example put this in the include dir?
#include "tf_keyboard_cal_gui.h"

namespace tf_keyboard_cal
{

std::vector< tf_data > active_tf_list_;

TFKeyboardCalGui::TFKeyboardCalGui(QWidget* parent) : rviz::Panel(parent)
{ 
  tab_widget_ = new QTabWidget;
  tab_widget_->addTab(new createTFTab(), tr("Add / Remove"));

  new_manipulate_tab_ = new manipulateTFTab();
  connect(tab_widget_, SIGNAL(tabBarClicked(int)), this, SLOT(updateTFList()));
  tab_widget_->addTab(new_manipulate_tab_, tr("Manipulate"));
  // tab_widget_->addTab(new manipulateTFTab(), tr("Manipulate"));
  
  tab_widget_->addTab(new saveLoadTFTab(), tr("Save / Load"));

  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addWidget(tab_widget_);
  setLayout(main_layout);
}

void TFKeyboardCalGui::updateTFList()
{
  new_manipulate_tab_->updateTFList();
}

createTFTab::createTFTab(QWidget *parent) : QWidget(parent)
{

  // TF controls
  QLabel *from_label = new QLabel(tr("from:"));
  QLabel *to_label = new QLabel(tr("to:"));
  
  from_ = new QComboBox;
  from_->setEditable(true);
  from_->addItem(tr("Select existing or add new TF"));
  connect(from_, SIGNAL(editTextChanged(const QString &)), this, SLOT(fromTextChanged(const QString &)));
  
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

  id_ = 0;
  
  remote_receiver_ = &TFRemoteReceiver::getInstance();
}

void createTFTab::createNewTF()
{
  ROS_DEBUG_STREAM_NAMED("createNewTF","create new TF button pressed.");
  ROS_DEBUG_STREAM_NAMED("createNewTF","from:to = " << from_tf_name_ << ":" << to_tf_name_);

  // create new tf
  tf_data new_tf;
  new_tf.id_ = id_++;
  new_tf.from_ = from_tf_name_;
  new_tf.to_ = to_tf_name_;
  for (std::size_t i = 0; i < 6; i++)
  {
    new_tf.values_[i] = 0.0;
  }
  std::string text = std::to_string(new_tf.id_) + ": " + new_tf.from_ + "-" + new_tf.to_;
  new_tf.name_ = QString::fromStdString(text);
  active_tf_list_.push_back(new_tf);

  // repopulate dropdown box
  active_tfs_->clear();
  for (std::size_t i = 0; i < active_tf_list_.size(); i++)
  {
    active_tfs_->addItem(active_tf_list_[i].name_);
  }

  // publish new tf
  remote_receiver_->createTF(new_tf.getTFMsg());
  
  // update from list
  from_->clear();
  from_->addItem(tr("Select existing or add new TF"));
  std::list<std::string> names;
  for (std::size_t i = 0; i < active_tf_list_.size(); i++)
  {
    names.push_back(active_tf_list_[i].from_);
    names.push_back(active_tf_list_[i].to_);
  }

  names.sort();
  names.unique();

  std::list<std::string>::iterator it;
  for (it = names.begin(); it != names.end(); ++it)
  {
    from_->addItem(tr( (*it).c_str() ));
  }
}

geometry_msgs::TransformStamped tf_data::getTFMsg()
{
  geometry_msgs::TransformStamped msg;
  msg.header.frame_id = from_;
  msg.header.stamp = ros::Time::now();
  msg.child_frame_id = to_;
  msg.transform.translation.x = values_[0];
  msg.transform.translation.y = values_[1];
  msg.transform.translation.z = values_[2];

  tf::Quaternion q;
  double deg_to_rad = 3.14159265 / 180.0;
  q.setRPY(values_[3] * deg_to_rad, values_[4] * deg_to_rad, values_[5] * deg_to_rad);

  msg.transform.rotation.x = q[0];
  msg.transform.rotation.y = q[1];
  msg.transform.rotation.z = q[2];
  msg.transform.rotation.w = q[3];
  
  return msg;
}

void createTFTab::removeTF()
{
  // find tf and remove from list
  std::string tf_id = active_tfs_->currentText().toStdString();
  ROS_DEBUG_STREAM_NAMED("removeTF","remove TF: " << active_tfs_->currentIndex() << ", " << tf_id);
  for (std::size_t i = 0; i < active_tf_list_.size(); i++)
  {
    if (active_tf_list_[i].name_ == active_tfs_->currentText())
    {
      ROS_DEBUG_STREAM_NAMED("removeTF","remove index = " << i);
      remote_receiver_->removeTF(active_tf_list_[i].getTFMsg());
      active_tf_list_.erase(active_tf_list_.begin() + i);
      break;
    }
  }

  // repopulate dropdown lists
  active_tfs_->clear();
  for (std::size_t i = 0; i < active_tf_list_.size(); i++)
  {
    active_tfs_->addItem(active_tf_list_[i].name_);
  }

  // update from list
  from_->clear();
  from_->addItem(tr("Select existing or add new TF"));
  std::list<std::string> names;
  for (std::size_t i = 0; i < active_tf_list_.size(); i++)
  {
    names.push_back(active_tf_list_[i].from_);
    names.push_back(active_tf_list_[i].to_);
  }

  names.sort();
  names.unique();

  std::list<std::string>::iterator it;
  for (it = names.begin(); it != names.end(); ++it)
  {
    from_->addItem(tr( (*it).c_str() ));
  }
  
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
  QLabel *tf_label = new QLabel(tr("tf:"));
  active_tfs_ = new QComboBox;
  active_tfs_->addItem(tr("Select TF"));
  connect(active_tfs_, SIGNAL(activated(int)), this, SLOT(setQLineValues(int)));

  QLabel *xyz_delta_label = new QLabel(tr("xyz delta (m):"));
  xyz_delta_box_ = new QLineEdit;
  xyz_delta_box_->setText("0.1");
  xyz_delta_ = 0.1;
  connect(xyz_delta_box_, SIGNAL(textChanged(const QString &)), this, SLOT(setXYZDelta(const QString &)));

  QLabel *rpy_delta_label = new QLabel(tr("rpy delta (deg):"));
  rpy_delta_box_ = new QLineEdit;
  rpy_delta_box_->setText("5.0");
  rpy_delta_ = 5.0;
  connect(rpy_delta_box_, SIGNAL(textChanged(const QString &)), this, SLOT(setRPYDelta(const QString &)));

  QGroupBox *tf_ctrl_section = new QGroupBox(tr("Manipulation Data"));

  QHBoxLayout *tf_row = new QHBoxLayout;
  tf_row->addWidget(tf_label);
  tf_row->addWidget(active_tfs_);

  QHBoxLayout *xyz_delta_row = new QHBoxLayout;
  xyz_delta_row->addWidget(xyz_delta_label);
  xyz_delta_row->addWidget(xyz_delta_box_);

  QHBoxLayout *rpy_delta_row = new QHBoxLayout;
  rpy_delta_row->addWidget(rpy_delta_label);
  rpy_delta_row->addWidget(rpy_delta_box_);
  
  QVBoxLayout *tf_controls = new QVBoxLayout;
  tf_controls->addLayout(tf_row);
  tf_controls->addLayout(xyz_delta_row);
  tf_controls->addLayout(rpy_delta_row);
  tf_ctrl_section->setLayout(tf_controls);

  // Set up xyr rpy increment controls
  QGroupBox *tf_increment_section = new QGroupBox(tr("Increment TF"));
  QGridLayout *grid_layout = new QGridLayout();
  
  std::vector<std::string> labels = { "x (m):", "y (m):", "z (m):", "r (deg):", "p (deg):", "y (deg):"};
  for (std::size_t i = 0; i < labels.size(); i++)
  { 
    QLabel *label = new QLabel(tr(labels[i].c_str()));
    
    QPushButton *minus = new QPushButton;
    minus->setText("-");
    minus->setProperty("sign", -1.0);
    minus->setProperty("dof", (int)i);
    connect(minus, SIGNAL(clicked()), this, SLOT(incrementDOF()));
    
    QLineEdit *value = new QLineEdit;
    value->setText("0.0");
    value->setProperty("dof", (int)i);
    dof_qline_edits_.push_back(value);
    connect(dof_qline_edits_[i], SIGNAL(textEdited(const QString &)), this, SLOT(editTFTextValue(const QString &)));

    QPushButton *plus = new QPushButton;
    plus->setText("+");
    plus->setProperty("sign", 1.0);
    plus->setProperty("dof", (int)i);    
    connect(plus, SIGNAL(clicked()), this, SLOT(incrementDOF()));

    grid_layout->addWidget(label, i, 1);
    grid_layout->addWidget(minus, i, 2);
    grid_layout->addWidget(value, i, 3);
    grid_layout->addWidget(plus, i, 4);
  }

  tf_increment_section->setLayout(grid_layout);

  // set main layout
  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addWidget(tf_ctrl_section);
  main_layout->addWidget(tf_increment_section);
  setLayout(main_layout);

  remote_receiver_ = &TFRemoteReceiver::getInstance();
}

void manipulateTFTab::setQLineValues(int item_id)
{
  for (std::size_t i = 0; i < active_tf_list_.size(); i++)
  {
    if (active_tf_list_[i].name_ == active_tfs_->currentText())
    {
      ROS_DEBUG_STREAM_NAMED("setQLineValues","name_ = " << active_tf_list_[i].name_.toStdString());
      for (std::size_t j = 0; j < 6; j++)
      {
        dof_qline_edits_[j]->setText(QString::number(active_tf_list_[i].values_[j]));
      }
      break;
    }
  }
  
}

void manipulateTFTab::updateTFList()
{
  active_tfs_->clear();
  for (std::size_t i = 0; i < active_tf_list_.size(); i++)
  {
    active_tfs_->addItem(active_tf_list_[i].name_);
  }

  setQLineValues(active_tfs_->currentIndex());
}

void manipulateTFTab::editTFTextValue(QString text)
{
  double value = text.toDouble();
  int dof = (sender()->property("dof")).toInt();
  
  updateTFValues(dof, value);
}

void manipulateTFTab::incrementDOF()
{
  int dof = (sender()->property("dof")).toInt();
  double sign = (sender()->property("sign")).toDouble();

  double value = dof_qline_edits_[dof]->text().toDouble();
  
  if (dof == 0 || dof == 1 || dof == 2)
    value += (double)sign * xyz_delta_;
  else
    value += (double)sign * rpy_delta_;

  dof_qline_edits_[dof]->setText(QString::number(value));
  updateTFValues(dof, value);  
}

void manipulateTFTab::updateTFValues(int dof, double value)
{
  for (std::size_t i = 0; i < active_tf_list_.size(); i++)
  {
    if (active_tf_list_[i].name_ == active_tfs_->currentText())
    {
      ROS_DEBUG_STREAM_NAMED("updateTFValues","name_ = " << active_tf_list_[i].name_.toStdString());
      active_tf_list_[i].values_[dof] = value;
      for (std::size_t j = 0; j < 6; j++)
      {
        ROS_DEBUG_STREAM_NAMED("updateTFValues", j << " = " << active_tf_list_[i].values_[j]);
      }
      remote_receiver_->updateTF(active_tf_list_[i].getTFMsg());
      break;
    }
  }
}

void manipulateTFTab::setXYZDelta(QString text)
{
  xyz_delta_ = text.toDouble();
  ROS_DEBUG_STREAM_NAMED("set_xyz_delta","text = " << text.toStdString() << ", value = " << xyz_delta_);  

  if (std::abs(xyz_delta_) > MAX_XYZ_DELTA)
  {
    ROS_WARN_STREAM_NAMED("setXYZDelta","Tried to set XYZ delta outside of limits. (+/-" << MAX_XYZ_DELTA << ")");
    xyz_delta_ > 0 ? xyz_delta_ = MAX_XYZ_DELTA : xyz_delta_ = -1.0 * MAX_XYZ_DELTA;
    QString value = QString::number(xyz_delta_);
    xyz_delta_box_->setText(value);
  }
  
  ROS_DEBUG_STREAM_NAMED("set_xyz_delta","setting xyz_delta_ = " << xyz_delta_);
}

void manipulateTFTab::setRPYDelta(QString text)
{
  rpy_delta_ = text.toDouble();
  ROS_DEBUG_STREAM_NAMED("set_rpy_delta","text = " << text.toStdString() << ", value = " << rpy_delta_);

  if (std::abs(rpy_delta_) > MAX_RPY_DELTA)
  {
    ROS_WARN_STREAM_NAMED("setRPYDelta","Tried to set RPY delta outside of limits. (+/-" << MAX_RPY_DELTA << ")");
    rpy_delta_ > 0 ? rpy_delta_ = MAX_RPY_DELTA : rpy_delta_ = -1.0 * MAX_RPY_DELTA;
    QString value = QString::number(rpy_delta_);
    rpy_delta_box_->setText(value);
  }
  
  ROS_DEBUG_STREAM_NAMED("set_rpy_delta","setting rpy_delta_ = " << rpy_delta_);
}

saveLoadTFTab::saveLoadTFTab(QWidget *parent) : QWidget(parent)
{
  load_btn_ = new QPushButton(tr("Load TFs from File"), this);
  connect(load_btn_ , SIGNAL(clicked()), this, SLOT(load()));

  save_btn_ = new QPushButton(tr("Save TFs to File"), this);
  connect(save_btn_ , SIGNAL(clicked()), this, SLOT(save()));

  QGroupBox *file_section = new QGroupBox(tr("Save/Load TF Files"));
  
  QVBoxLayout *file_layout = new QVBoxLayout;
  file_layout->addWidget(load_btn_);
  file_layout->addWidget(save_btn_);

  file_section->setLayout(file_layout);

  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addWidget(file_section);
  setLayout(main_layout);

}

void saveLoadTFTab::load()
{
  QString filters("YAML files (*.yaml);;All files (*.*)");
  QString default_filter("YAML files (*.yaml)");
  
  QString directory =
    QFileDialog::getOpenFileName(0, "Load TF File", QDir::currentPath(), filters, &default_filter);

  full_load_path_ = directory.toStdString();
  ROS_DEBUG_STREAM_NAMED("load","load_file = " << full_load_path_);
}

void saveLoadTFTab::save()
{
  QString filters("YAML files (*.yaml);;All files (*.*)");
  QString default_filter("YAML files (*.yaml)");
  
  QString directory =
    QFileDialog::getSaveFileName(0, "Save TF File", QDir::currentPath(), filters, &default_filter);


  full_save_path_ = directory.toStdString();

  // TODO: check for file extension and automatically append if left off
  
  ROS_DEBUG_STREAM_NAMED("load","save_file = " << full_save_path_);
}

} // end namespace tf_keyboard_cal

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tf_keyboard_cal::TFKeyboardCalGui, rviz::Panel)
