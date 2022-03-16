/*!*******************************************************************************************
 *  \file       execution_viewer.h
 *  \brief      Execution_viewer definition file.
 *  \details    This file displays the behaviors and beliefs used while the tree is executing.
 *  \author     Jorge Luis Pascual, Carlos Valencia.
 *  \copyright   Copyright 2019 Universidad Politecnica de Madrid (UPM)
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/
#ifndef BEHAVIOREXECUTIONVIEWER_H
#define BEHAVIOREXECUTIONVIEWER_H

#include <ros/ros.h>
#include <aerostack_msgs/ListOfBeliefs.h>
#include <aerostack_msgs/ListOfRunningTasks.h>
#include <aerostack_msgs/ActivationChange.h>
#include <aerostack_msgs/StopTask.h>
//#include <aerostack_msgs/ExecutionRequest.h>
#include <thread>
#include <chrono>
#include <sstream>
#include <aerostack_msgs/TaskStopped.h>
#include <QWidget>
#include <QtWidgets>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QGridLayout>
#include <QTextEdit>
#include <QMenu>
#include <QLabel>
#include <QSize>
#include <QCursor>
#include <QString>
#include <QRect>
#include <QGuiApplication>
#include <QScreen>
#include <QProcess>
#include <QColor>
#include <QBrush>

#include "ui_behavior_execution_viewer.h"
#include "behavior_execution_viewer_dialog.h"

#include "yaml-cpp/yaml.h"
#include <QSizePolicy>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/algorithm/remove_if.hpp>
#include "fstream"

namespace Ui
{
class BehaviorExecutionViewer;

}
class BehaviorExecutionViewer : public QWidget
{
  Q_OBJECT
public:
  explicit BehaviorExecutionViewer(int argc, char** argv, QWidget* parent = 0);
  ~BehaviorExecutionViewer();

  QTableWidget* getActiveBehaviorExecutionViewer();
  /*!********************************************************************************************************************
   *  \brief      This method refreshes the list of behaviors
   **********************************************************************************************************************/
  void executionRequestCallback(const aerostack_msgs::ActivationChange& msg);
  int position_x=405;
  int position_y=-395;

  std::list<QTableWidgetItem> list_active_tasks;

private:
  Ui::BehaviorExecutionViewer* ui;
   std::string catalog_path;
  ros::NodeHandle n;
  ros::ServiceClient cancel_behavior_srv;
  ros::Subscriber list_of_behaviors_sub;
  ros::Subscriber list_of_running_tasks_sub;
  ros::Subscriber behavior_sub;
  QGridLayout* my_layout;
  QLabel* active_behavior_label;
  QLabel* behavior_label;
  QTableWidget* behavior_content;
  QTableWidget* active_behavior_content;
  QTextEdit* t;
  QTableWidgetItem* behavior_top_level_item;
  QPoint* point;
  QMenu* behavior_context_menu;
  std::string task_stopped;
  bool is_behavior_context_menu_created;
  int a = 0;//para las filas de la tabla
  int activeBehaviorCount =0;
  std::string cancel_behavior;
  std::vector<std::string> all_behaviors_active;
  std::string drone_id_namespace;
  std::string list_of_running_tasks;
  std::string execution_request;
  std::string activation_change;

  ros::Publisher window_event_pub;
  ros::Subscriber window_event_sub;

  std::string window_event_topic;


  boost::property_tree::ptree root;

  /*!********************************************************************************************************************
   *  \brief      This method sets up the necessary characteristics for the Behavior list to work properly
   **********************************************************************************************************************/
  void setUpBehaviorListTable();

  void setUpBehaviorListTableRows();

  void clearBehaviorTable();

  void resizeEvent(QResizeEvent* event);

  /*!********************************************************************************************************************
   *  \brief      This method is the responsible for seting up connections.
   *********************************************************************************************************************/
  void setUp();
  
private Q_SLOTS:
  void cellClickedR(int nRow, int nCol);
 
public Q_SLOTS:
  /*!********************************************************************************************************************
   *  \brief   This slot is executed when the user right clicks anywhere inside the behaviors tree's widget.
   **********************************************************************************************************************/
  void behaviorCustomContextMenu(const QPoint&);

  /*!********************************************************************************************************************
   *  \brief      This slot is executed when the user wants to add a behavior
   **********************************************************************************************************************/
  void addBehavior();

  /*!********************************************************************************************************************
   *  \brief      This slot is executed when the user wants to stop the execution of a behavior
   **********************************************************************************************************************/
  void stopBehavior();

  /*!********************************************************************************************************************
   *  \brief      This method notifies main window that the widget was closed
   *********************************************************************************************************************/
  void closeEvent(QCloseEvent* event);

  /*!************************************************************************
   *  \brief   Activated when a window is closed.
   ***************************************************************************/
  //void windowOpenCallback(const aerostack_msgs::WindowEvent& msg);
  /*!************************************************************************
   *  \brief  Kills the process
   ***************************************************************************/
  void killMe();

  /*!********************************************************************************************************************
   *  \brief   This method launch a window which contains detailed error
   **********************************************************************************************************************/
  void windowFail(std::string title, std::string message);

  /*!********************************************************************************************************************
   *  \brief   This method launch a window which contains detailed info
   **********************************************************************************************************************/
  

Q_SIGNALS:
  /*!********************************************************************************************************************
   *  \brief      This signal is emitted when new behaviors info is received
   **********************************************************************************************************************/
  void setBehaviorText(const QString&);

};

#endif
