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

// TODO: Why doesn't example put this in the include dir?
// Looks like it doesn't work if *.h is in include dir?
#include "tf_keyboard_cal_gui.h"

namespace tf_keyboard_cal
{

std::vector< tf_data > active_tf_list_;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imarker_server_;
interactive_markers::MenuHandler imarker_menu_handler_;

TFKeyboardCalGui::TFKeyboardCalGui(QWidget* parent) : rviz::Panel(parent)
{ 
  tab_widget_ = new QTabWidget;
  connect(tab_widget_, SIGNAL(tabBarClicked(int)), this, SLOT(updateTabData(int)));
  
  new_create_tab_ = new createTFTab();
  tab_widget_->addTab(new_create_tab_, tr("Add / Remove"));

  new_manipulate_tab_ = new manipulateTFTab();
  tab_widget_->addTab(new_manipulate_tab_, tr("Manipulate"));

  new_save_load_tab_ = new saveLoadTFTab(); 
  tab_widget_->addTab(new_save_load_tab_, tr("Save / Load"));

  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addWidget(tab_widget_);
  setLayout(main_layout);

  imarker_server_.reset(new interactive_markers::InteractiveMarkerServer("tf_keyboard_cal_imarkers", "", false));
  ros::Duration(0.25).sleep();
                        
  updateTabData(0);
}

void TFKeyboardCalGui::updateTabData(int index)
{
  ROS_DEBUG_STREAM_NAMED("updateTabData","index = " << index);
  new_manipulate_tab_->updateTFList();
  new_create_tab_->updateFromList();

  if (index == 1) // manipulate tab selected. 
  {
    new_manipulate_tab_->setFocus();
  }
}

createTFTab::createTFTab(QWidget *parent) : QWidget(parent)
{
  menu_handler_set_ = false;
  
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

  add_imarker_ = new QCheckBox("i marker?", this);
  add_imarker_->setCheckState(Qt::Unchecked);

  add_imarker_menu_ = new QCheckBox("menus?", this);
  add_imarker_menu_->setCheckState(Qt::Unchecked); 
  
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

  QHBoxLayout *create_row = new QHBoxLayout;
  create_row->addWidget(add_imarker_);
  create_row->addWidget(add_imarker_menu_);
  create_row->addWidget(create_tf_btn_);
  
  QHBoxLayout *remove_row = new QHBoxLayout;
  remove_row->addWidget(active_tfs_);
  remove_row->addWidget(remove_tf_btn_);

  QVBoxLayout *add_controls = new QVBoxLayout;
  add_controls->addLayout(from_row);
  add_controls->addLayout(to_row);
  add_controls->addLayout(create_row);
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

  this->setFocus();
  
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

  // interactive marker
  new_tf.imarker_ = false;
  if (add_imarker_->isChecked())
  {
    new_tf.imarker_ = true;
    ROS_DEBUG_STREAM_NAMED("createNewTF","imarker = " << new_tf.imarker_);
    createNewIMarker(new_tf, add_imarker_menu_->isChecked());
  }
  active_tf_list_.push_back(new_tf);
  
  // repopulate dropdown box
  active_tfs_->clear();
  for (std::size_t i = 0; i < active_tf_list_.size(); i++)
  {
    active_tfs_->addItem(active_tf_list_[i].name_);
  }

  // publish new tf
  remote_receiver_->createTF(new_tf.getTFMsg());

  updateFromList();  
}

void createTFTab::createNewIMarker(tf_data new_tf, bool has_menu)
{
  ROS_DEBUG_STREAM_NAMED("createNewIMarker","creating interactive marker...");
  
  // create a box to visualize the marker
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = new_tf.from_;
  int_marker.scale = 0.25;
  int_marker.name = new_tf.name_.toStdString();

  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(marker);
  box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  int_marker.controls.push_back(box_control);

  // create the handles to control individual dofs
  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  
  imarker_server_->insert(int_marker);
  imarker_server_->setCallback(int_marker.name, boost::bind( &createTFTab::processIMarkerFeedback, this, _1) );
  
  if (has_menu && !menu_handler_set_)
  {
    ROS_DEBUG_STREAM_NAMED("createNewIMarker","set menu");

    QString filters("rviz tf files (*.tf);;All files (*.*)");
    QString default_filter("rviz tf files (*.tf)");
  
    QString directory =
      QFileDialog::getOpenFileName(0, "Load TF Menu File", QDir::currentPath(), filters, &default_filter);

    std::string full_load_path;
    full_load_path = directory.toStdString();
    ROS_DEBUG_STREAM_NAMED("load","load_file = " << full_load_path);

    std::ifstream in_file(full_load_path);
    std::string line;
    int result;
    int level;
    char name[256];
    std::string menu_name(name);
    
    if (in_file.is_open())
    {
      while (std::getline(in_file, line))
      {
        ROS_DEBUG_STREAM_NAMED("createNewIMarker",line);
        result = sscanf(line.c_str(), "%d, %[^\t\n]s", &level, name);
        ROS_DEBUG_STREAM_NAMED("createNewIMarker","result = " << result);
        if (result == 2)
        {
          std::string menu_name(name);
          if (level == 1)
            imarker_menu_handler_.insert(name, boost::bind(&createTFTab::processIMarkerFeedback, this, _1));
        }
      }
    }
    imarker_menu_handler_.apply(*imarker_server_, int_marker.name);
    menu_handler_set_ = true;
  }
  
  imarker_server_->applyChanges();
}

void createTFTab::processIMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_DEBUG_STREAM_NAMED("processIMarkerFeedback","Feedback from " << feedback->marker_name);

  QString imarker_name = QString::fromStdString(feedback->marker_name);
  
  for (std::size_t i = 0; i < active_tf_list_.size(); i++)
  {
    if (active_tf_list_[i].name_ == imarker_name)
    {
      ROS_DEBUG_STREAM_NAMED("removeTF","update index = " << i);
      manipulateTFTab::updateTFValues(i, feedback->pose);
      remote_receiver_->updateTF(active_tf_list_[i].getTFMsg());
      break;
    }
  }  
}

void createTFTab::updateFromList()
{
  // give tf a chance to update
  ros::Duration(0.25).sleep();
  
  // update from list
  from_->clear();
  from_->addItem(tr("Select existing or add new TF"));
  std::vector<std::string> names = remote_receiver_->getTFNames();
  for (std::size_t i = 0; i < names.size(); i++)
  {
    from_->addItem(tr(names[i].c_str()));
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

void manipulateTFTab::keyPressEvent(QKeyEvent *event)
{

  double xyz_delta = 0.01;
  double rpy_delta = 1.0; // degrees
  
  switch (event->key())
  {
    case Qt::Key_A:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","x+");
      incrementDOF(0, 1.0);
      break;
    case Qt::Key_Q:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","x-");
      incrementDOF(0, -1.0);
      break;
    case Qt::Key_W:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","y+");
      incrementDOF(1, 1.0);
      break;
    case Qt::Key_S:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","y-");
      incrementDOF(1, -1.0);
      break;
    case Qt::Key_E:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","z+");
      incrementDOF(2, 1.0);
      break;
    case Qt::Key_D:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","z-");
      incrementDOF(2, -1.0);
      break;
    case Qt::Key_R:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","roll+");
      incrementDOF(3, 1.0);
      break;
    case Qt::Key_F:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","roll-");
      incrementDOF(3, -1.0);
      break;
    case Qt::Key_T:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","pitch+");
      incrementDOF(4, 1.0);
      break;
    case Qt::Key_G:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","pitch-");
      incrementDOF(4, -1.0);
      break;
    case Qt::Key_Y:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","yaw+");
      incrementDOF(5, 1.0);
      break;
    case Qt::Key_H:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","yaw-");
      incrementDOF(5, -1.0);
      break;
    case Qt::Key_U:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","fast");
      xyz_delta = 0.1;
      rpy_delta = 5.0; // degrees
      xyz_delta_box_->setText(QString::number(xyz_delta));
      rpy_delta_box_->setText(QString::number(rpy_delta));
      setXYZDelta(xyz_delta);
      setRPYDelta(rpy_delta);
      break;
    case Qt::Key_I:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","medium");
      xyz_delta = 0.01;
      rpy_delta = 1.0; // degrees
      xyz_delta_box_->setText(QString::number(xyz_delta));
      rpy_delta_box_->setText(QString::number(rpy_delta));      
      setXYZDelta(xyz_delta);
      setRPYDelta(rpy_delta);      
      break;
    case Qt::Key_O:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","slow");
      xyz_delta = 0.001;
      rpy_delta = 0.5; // degrees
      xyz_delta_box_->setText(QString::number(xyz_delta));
      rpy_delta_box_->setText(QString::number(rpy_delta));      
      setXYZDelta(xyz_delta);
      setRPYDelta(rpy_delta);
      break;
    default:
      ROS_DEBUG_STREAM_NAMED("keyPressEvent","undefined key pressed");
      break;
  }
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

  incrementDOF(dof, sign);
}

void manipulateTFTab::incrementDOF(int dof, double sign)
{
  double value = dof_qline_edits_[dof]->text().toDouble();
  
  if (dof == 0 || dof == 1 || dof == 2)
    value += (double)sign * xyz_delta_;
  else
    value += (double)sign * rpy_delta_;

  dof_qline_edits_[dof]->setText(QString::number(value));
  updateTFValues(dof, value);  
}

void manipulateTFTab::updateTFValues(int idx, geometry_msgs::Pose pose)
{
  // TODO: update euler angles, change pose -> transformstamped, call remote_receiver_, update gui text
  active_tf_list_[idx].values_[0] = pose.position.x;
  active_tf_list_[idx].values_[1] = pose.position.y;
  active_tf_list_[idx].values_[2] = pose.position.z;

  double rad_to_deg = 180.0 / 3.14159265;
  
  Eigen::Quaterniond eigen_quaternion;
  tf::quaternionMsgToEigen(pose.orientation, eigen_quaternion);
  Eigen::Vector3d euler_angles = eigen_quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
  ROS_DEBUG_STREAM_NAMED("updateTFValues","euler_angles = " << euler_angles.transpose());
  active_tf_list_[idx].values_[3] = euler_angles[0] * rad_to_deg;
  active_tf_list_[idx].values_[4] = euler_angles[1] * rad_to_deg;
  active_tf_list_[idx].values_[5] = euler_angles[2] * rad_to_deg;
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

      geometry_msgs::TransformStamped tf_msg = active_tf_list_[i].getTFMsg();
      ROS_DEBUG_STREAM_NAMED("updateTFValues","imarker_ = " << active_tf_list_[i].imarker_);
      if (active_tf_list_[i].imarker_)
      {
        ROS_DEBUG_STREAM_NAMED("updateTFValues","update imarker pose...");
        geometry_msgs::Pose imarker_pose;
        imarker_pose.position.x = tf_msg.transform.translation.x;
        imarker_pose.position.y = tf_msg.transform.translation.y;
        imarker_pose.position.z = tf_msg.transform.translation.z;
        imarker_pose.orientation = tf_msg.transform.rotation;
        imarker_server_->setPose(active_tf_list_[i].name_.toStdString(), imarker_pose);
        imarker_server_->applyChanges();
      }
      
      remote_receiver_->updateTF(tf_msg);
      break;
    }
  }
}

void manipulateTFTab::setXYZDelta(QString text)
{ 
  double xyz_delta = text.toDouble();

  ROS_DEBUG_STREAM_NAMED("set_xyz_delta","text = " << text.toStdString() << ", value = " << xyz_delta);
  
  setXYZDelta(xyz_delta);
}

void manipulateTFTab::setXYZDelta(double xyz_delta)
{
  xyz_delta_ = xyz_delta;
  
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
  double rpy_delta = text.toDouble();
  
  ROS_DEBUG_STREAM_NAMED("set_rpy_delta","text = " << text.toStdString() << ", value = " << rpy_delta);

  setRPYDelta(rpy_delta);
}

void manipulateTFTab::setRPYDelta(double rpy_delta)
{
  rpy_delta_ = rpy_delta;
  
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

  remote_receiver_ = &TFRemoteReceiver::getInstance();
}

void saveLoadTFTab::load()
{
  QString filters("rviz tf files (*.tf);;All files (*.*)");
  QString default_filter("rviz tf files (*.tf)");
  
  QString directory =
    QFileDialog::getOpenFileName(0, "Load TF File", QDir::currentPath(), filters, &default_filter);

  full_load_path_ = directory.toStdString();
  ROS_DEBUG_STREAM_NAMED("load","load_file = " << full_load_path_);

  std::ifstream in_file(full_load_path_);
  std::string line;

  int id;
  int result;
  char from[256];
  char to[256];
  float x, y, z, roll, pitch, yaw;
  
  if (in_file.is_open())
  {
    while (std::getline(in_file, line))
    {
      result = sscanf(line.c_str(), "%d %s %s %f %f %f %f %f %f", &id, from, to, &x, &y, &z, &roll, &pitch, &yaw);

      if (result == 9)
      {
        ROS_DEBUG_STREAM_NAMED("load",id);
        ROS_DEBUG_STREAM_NAMED("load",from);
        ROS_DEBUG_STREAM_NAMED("load",to);
        ROS_DEBUG_STREAM_NAMED("load",x);
        ROS_DEBUG_STREAM_NAMED("load",y);
        ROS_DEBUG_STREAM_NAMED("load",z);
        ROS_DEBUG_STREAM_NAMED("load",roll);
        ROS_DEBUG_STREAM_NAMED("load",pitch);
        ROS_DEBUG_STREAM_NAMED("load",yaw);
        
        // create new tf
        tf_data new_tf;
        new_tf.id_ = id;
        std::string frame_id(from);
        new_tf.from_ = frame_id;
        std::string child_frame_id(to);
        new_tf.to_ = child_frame_id;
        new_tf.values_[0] = x;
        new_tf.values_[1] = y;
        new_tf.values_[2] = z;
        new_tf.values_[3] = roll;
        new_tf.values_[4] = pitch;
        new_tf.values_[5] = yaw;
       
        std::string text = std::to_string(new_tf.id_) + ": " + new_tf.from_ + "-" + new_tf.to_;
        new_tf.name_ = QString::fromStdString(text);
        
        active_tf_list_.push_back(new_tf);
        remote_receiver_->createTF(new_tf.getTFMsg());
      }
      else
      {
        ROS_INFO_STREAM_NAMED("load","skipping line: " << line);
      }
    }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("load","Unable to open file: " << full_load_path_);
  }
  in_file.close();
}

void saveLoadTFTab::save()
{
  QString filters("rviz tf files (*.tf);;All files (*.*)");
  QString default_filter("rviz tf files (*.tf)");
  
  QString directory =
    QFileDialog::getSaveFileName(0, "Save TF File", QDir::currentPath(), filters, &default_filter);

  full_save_path_ = directory.toStdString();

  // check if user specified file extension
  std::size_t found = full_save_path_.find(".");
  if (found == std::string::npos)
    full_save_path_ += ".tf";
  
  ROS_DEBUG_STREAM_NAMED("save","save_file = " << full_save_path_);

  std::ofstream out_file;
  out_file.open(full_save_path_);
  
  std::string header = "#ID FRAME_ID CHILD_FRAME_ID X Y Z ROLL PITCH YAW\n# (comment line)\n# space delimeter";

  if (out_file.is_open())
  {
    out_file << header << std::endl;
    for (std::size_t i = 0; i < active_tf_list_.size(); i++)
    {
      out_file << active_tf_list_[i].id_ << " ";
      out_file << active_tf_list_[i].from_ << " ";
      out_file << active_tf_list_[i].to_;
      for (std::size_t j = 0; j < 6; j++)
      {
        out_file << " " << active_tf_list_[i].values_[j];
      }
      out_file << std::endl;
    }
    out_file.close();
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("save","Unable to open file: " << full_save_path_);
  }
}

} // end namespace tf_keyboard_cal

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tf_keyboard_cal::TFKeyboardCalGui, rviz::Panel)
