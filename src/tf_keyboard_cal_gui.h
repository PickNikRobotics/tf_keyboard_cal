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
 * Created   : 25 - April - 2017
 */

#ifndef TF_KEYBOARD_CAL_GUI_H
#define TF_KEYBOARD_CAL_GUI_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <QTabWidget>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QFrame>

namespace tf_keyboard_cal
{

class TFKeyboardCalGui : public rviz::Panel
{
  Q_OBJECT
public:
  explicit TFKeyboardCalGui(QWidget *parent = 0);

private:
  QTabWidget *tabWidget_;  
};

/**
 * Tab for creating, saving, loading, removing and deleting TFs
 */
class createTFTab : public QWidget
{
  Q_OBJECT

public:
  explicit createTFTab(QWidget *parent = 0);

protected Q_SLOTS:
  void createNewTF();
  void removeTF();

  void fromTextChanged(QString text);
  void toTextChanged(QString text);
  
private:
  std::string from_tf_name_;
  std::string to_tf_name_;

  QLineEdit *from_;
  QLineEdit *to_;

  QPushButton *create_tf_btn_;
  QPushButton *remove_tf_btn_;

  QComboBox *active_tfs_;
};

/**
 * Tab for manipulating TFs
 */
class manipulateTFTab : public QWidget
{
  Q_OBJECT

public:
  explicit manipulateTFTab(QWidget *parent = 0);
};

/**
 * Tab for saving and loading TFs
 */
class saveLoadTFTab : public QWidget
{
  Q_OBJECT

public:
  explicit saveLoadTFTab(QWidget *parent = 0);
};

} // end namespace tf_keyboard_cal

#endif
