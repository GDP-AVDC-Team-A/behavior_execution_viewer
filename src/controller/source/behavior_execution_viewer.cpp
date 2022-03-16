/*!*******************************************************************************************
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
/*
  BehaviorExecutionViewer
  @author  Jorge Luis Pascual, Carlos Valencia.
  @date    07-2017
  @version 2.0
*/
#include "../include/behavior_execution_viewer.h"
#include "../include/global.h"

BehaviorExecutionViewer::BehaviorExecutionViewer(int argc, char** argv, QWidget* parent) : QWidget(parent), ui(new Ui::BehaviorExecutionViewer)
{
  QWidget::setLocale(QLocale());

  ui->setupUi(this);

  // window always on top
  Qt::WindowFlags flags = windowFlags();
  setWindowFlags(flags | Qt::WindowStaysOnTopHint);
  setWindowIcon(QIcon(":/img/img/execution_viewer.png"));
  setWindowTitle("Behavior Execution Viewer");
  qRegisterMetaType<QVector<int>>("QVector<int>");
  my_layout = ui->gridLayout;
  this->point = 0;
  
  active_behavior_label = new QLabel("Active behaviors", this);
  behavior_label = new QLabel("Execution sequence", this);

  active_behavior_content = new QTableWidget(this);
  active_behavior_content->setAlternatingRowColors(true);

  behavior_content = new QTableWidget(this);
  behavior_content->setAlternatingRowColors(true);

  QSizePolicy behavior_policy = QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  behavior_policy.setVerticalStretch(1);
  behavior_content->setSizePolicy(behavior_policy);
  active_behavior_content->setSizePolicy(behavior_policy);
  active_behavior_content->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  my_layout->addWidget(active_behavior_label, 0, 0);
  my_layout->addWidget(active_behavior_content, 1, 0);
  active_behavior_label->show();
  active_behavior_content->show();

  my_layout->addWidget(behavior_label, 2, 0);
  my_layout->addWidget(behavior_content, 3, 0);
  behavior_label->show();
  behavior_content->show();
  this->is_behavior_context_menu_created = false;

  n.param<std::string>("cancel_behavior", cancel_behavior, "request_behavior_deactivation");
 ///n.param<std::string>("list_of_active_behaviors", list_of_active_behaviors, "list_of_active_behaviors");
  n.param<std::string>("list_of_running_tasks", list_of_running_tasks, "list_of_running_tasks");
  n.param<std::string>("robot_namespace", drone_id_namespace, "drone1");
  //n.param<std::string>("execution_request", execution_request, "execution_request");
  n.param<std::string>("activation_change", activation_change, "activation_change");
  n.param<std::string>("task_stopped", task_stopped, "task_stopped");


  cancel_behavior_srv = n.serviceClient<aerostack_msgs::RequestBehaviorDeactivation>(cancel_behavior);
  //list_of_running_tasks_sub = n.subscribe("/" + drone_id_namespace +  "/" + list_of_running_tasks, 1000, &BehaviorExecutionViewer::activeBehaviorCallback, this);
  list_of_behaviors_sub = n.subscribe("/" + drone_id_namespace + "/" + activation_change, 1000, &BehaviorExecutionViewer::executionRequestCallback, this);
  //behavior_sub=n.subscribe("/"+drone_id_namespace+"/"+task_stopped, 1000, &BehaviorExecutionViewer::behaviorCompletedCallback,this);
  setUpBehaviorListTable();

  // reads layout file
  //namespace pt = boost::property_tree;

  //std::string layout_dir = std::getenv("AEROSTACK_STACK") + std::string("/stack/ground_control_system/"  "graphical_user_interface/layouts/layout.json");

  //pt::read_json(layout_dir, root);

  QScreen* screen = QGuiApplication::primaryScreen();
  QRect screenGeometry = screen->geometry();

  int y0 = screenGeometry.height() / 2;
  int x0 = screenGeometry.width() / 2;
  int height =500; //root.get<int>("EXECUTION_VIEWER.height");
  int width = 500;//root.get<int>("EXECUTION_VIEWER.width");

  this->resize(width, height);
  this->move(x0 + position_x /*root.get<int>("EXECUTION_VIEWER.position.x")*/, y0 + position_y/*root.get<int>("EXECUTION_VIEWER.position.y")*/);
  // Settings connections
  setUp();
}

BehaviorExecutionViewer::~BehaviorExecutionViewer()
{
  delete ui;
}

QTableWidget* BehaviorExecutionViewer::getActiveBehaviorExecutionViewer()
{
  return active_behavior_content;
}

void BehaviorExecutionViewer::setUp()
{
  //n.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");
 // n.param<std::string>("window_event_topic", window_event_topic, "window_event");


  // Subscribers
  //window_event_sub =
      //n.subscribe("/" + drone_id_namespace + "/" + window_event_topic, 10, &BehaviorExecutionViewer::windowOpenCallback, this);

  // Publishers
  //window_event_pub =
      //n.advertise<aerostack_msgs::WindowEvent>("/" + drone_id_namespace + "/" + window_event_topic, 1, true);
}

void BehaviorExecutionViewer::executionRequestCallback(const aerostack_msgs::ActivationChange& msg)
{
  this->behavior_content->verticalScrollBar()->setSliderPosition(behavior_content->verticalScrollBar()->maximum());
  this->active_behavior_content->verticalScrollBar()->setSliderPosition(active_behavior_content->verticalScrollBar()->maximum());

  QFont font;
  font.setPointSize(9);
  QFont font2;
  font2.setPointSize(9);
  this->behavior_content->setEditTriggers(QAbstractItemView::NoEditTriggers);
  this->active_behavior_content->setEditTriggers(QAbstractItemView::NoEditTriggers);

  this->behavior_content->insertRow(a);
  if(this->behavior_content->rowCount() == 1){ //fila insertada para que se vea bien al scrolear
    this->behavior_content->insertRow(this->behavior_content->rowCount());
    this->behavior_content->setItem(a+1, 1, new QTableWidgetItem(QString::fromStdString(" ")));
  }

  // For each behavior active, add a child to the tree
  
  //Message info
  QTableWidgetItem* new_behavior = new QTableWidgetItem(QString::fromStdString(msg.task_behavior.behavior));
  QTableWidgetItem* new_behavior2 = new QTableWidgetItem(QString::fromStdString(msg.task_behavior.behavior));
  std::string str = msg.task_behavior.task_command.parameters;
  str.erase(boost::remove_if(str, boost::is_any_of("\'}{")), str.end());
  QTableWidgetItem* new_arguments = new QTableWidgetItem(QString::fromStdString(str));
  QTableWidgetItem* new_arguments2 = new QTableWidgetItem(QString::fromStdString(str));

  QTableWidgetItem* new_priority = new QTableWidgetItem(QString::number(msg.task_behavior.task_command.priority));
  //QTableWidgetItem* new_success = new QTableWidgetItem(msg.success);//Y
  //QTableWidgetItem* new_request_type = new QTableWidgetItem(QString::number(msg.change_type));//+ o -
  //QTableWidgetItem* new_failure_cause = new QTableWidgetItem(QString::fromStdString(msg.failure_cause));//N

  this->behavior_content->setItem(a, 0, new QTableWidgetItem(QString::number(a)));
  this->behavior_content->item(a, 0)->setFont(font);
  this->behavior_content->item(a, 0)-> setTextAlignment(Qt::AlignCenter);

  this->behavior_content->setItem(a, 1, new_behavior);
  this->behavior_content->item(a, 1)->setFont(font);

  this->behavior_content->setItem(a, 2, new_arguments);
  this->behavior_content->item(a, 2)->setFont(font);

  this->behavior_content->setItem(a, 3, new_priority);
  this->behavior_content->item(a, 3)->setFont(font);
  this->behavior_content->item(a, 3)-> setTextAlignment(Qt::AlignCenter);
  
  if(msg.change_type==aerostack_msgs::ActivationChange::REQUESTED_ACTIVATION|| msg.change_type== aerostack_msgs::ActivationChange::DEFAULT_ACTIVATION|| 
  msg.change_type==aerostack_msgs::ActivationChange::AUTOMATIC_ACTIVATION){ //activate
     this->behavior_content->setItem(a, 4, new QTableWidgetItem(QString::fromStdString("+")));
     this->behavior_content->item(a, 4)-> setTextAlignment(Qt::AlignCenter);
        this->active_behavior_content->setRowCount(activeBehaviorCount+1);

  all_behaviors_active.push_back(msg.task_behavior.behavior);

  this->active_behavior_content->setItem(activeBehaviorCount, 1,new_behavior2);
  this->active_behavior_content->item(activeBehaviorCount, 1)->setFont(font2);

  this->active_behavior_content->setItem(activeBehaviorCount, 2,new_arguments2);
  this->active_behavior_content->item(activeBehaviorCount, 2)->setFont(font2);
    activeBehaviorCount++;

  }
  /*else if(new_request_type->text().toInt() == 1){ //ACTIVATE=1
     this->behavior_content->setItem(a, 4, new QTableWidgetItem(QString::fromStdString("+")));
     this->behavior_content->item(a, 4)-> setTextAlignment(Qt::AlignCenter);
  }*/
  else{ //ACTIVATION_FINISHED=2

     this->behavior_content->setItem(a, 4, new QTableWidgetItem(QString::fromStdString("-")));
     this->behavior_content->item(a, 4)-> setTextAlignment(Qt::AlignCenter);

     int j =0;
	std::vector<std::string>::iterator it;
	bool found=false;
	for ( it = all_behaviors_active.begin() ; it != all_behaviors_active.end() && !found; ++it)
    {
    	if (*it ==msg.task_behavior.behavior ){
    		all_behaviors_active.erase(it);
    		found=true;
    	}
    	else{
    		    	j++;

    	}
    }
    active_behavior_content->removeRow(j);
	  activeBehaviorCount--;
  }

  if(msg.change_type==aerostack_msgs::ActivationChange::REQUESTED_ACTIVATION || msg.change_type==aerostack_msgs::ActivationChange::DEFAULT_ACTIVATION || msg.change_type==aerostack_msgs::ActivationChange::AUTOMATIC_ACTIVATION || 
  	msg.change_type==aerostack_msgs::ActivationChange::GOAL_ACHIEVED_SELF_DEACTIVATION || msg.change_type==aerostack_msgs::ActivationChange::AUTOMATIC_DEACTIVATION || 
  	msg.change_type==aerostack_msgs::ActivationChange::REQUESTED_DEACTIVATION){
     this->behavior_content->setItem(a, 5, new QTableWidgetItem(QString::fromStdString("Y")));
     this->behavior_content->item(a, 5)->setFont(font);
     this->behavior_content->item(a, 5)->setTextColor(Qt::darkGreen);
     this->behavior_content->item(a, 5)-> setTextAlignment(Qt::AlignCenter);
  }
  else{
     this->behavior_content->setItem(a, 5, new QTableWidgetItem(QString::fromStdString("N")));
     this->behavior_content->item(a, 5)->setTextColor(Qt::darkRed);
     //windowFail("Behavior information", msg.failure_cause);
     this->behavior_content->item(a, 5)-> setTextAlignment(Qt::AlignCenter);
  }

  a++;
}

void BehaviorExecutionViewer::setUpBehaviorListTable()
{
  QStringList* headers = new QStringList();
  headers->append("N");
  headers->append("Behavior Name");
  headers->append("Parameters");
  headers->append("P"); //priority
  headers->append("T"); //request_type
  headers->append("S"); //success

  this->active_behavior_content->setColumnCount(3);
  this->active_behavior_content->verticalHeader()->setVisible(true);
  this->active_behavior_content->verticalHeader()->setDefaultSectionSize(20);
  this->active_behavior_content->verticalHeader()->setOffsetToLastSection();
  this->active_behavior_content->verticalHeader()->hide();
  this->active_behavior_content->setHorizontalHeaderLabels(*headers);
  this->active_behavior_content->horizontalHeader()->setVisible(true);
  this->active_behavior_content->setSortingEnabled(false);
  this->active_behavior_content->setContextMenuPolicy(Qt::CustomContextMenu);
  this->active_behavior_content->viewport()->setFocusPolicy(Qt::NoFocus);
  connect(active_behavior_content, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(behaviorCustomContextMenu(const QPoint&)));

  this->behavior_content->setColumnCount(6);
  this->behavior_content->verticalHeader()->setVisible(true);
  this->behavior_content->verticalHeader()->setDefaultSectionSize(20);
  this->behavior_content->verticalHeader()->setOffsetToLastSection();
  this->behavior_content->verticalHeader()->hide();
  this->behavior_content->setHorizontalHeaderLabels(*headers);
  this->behavior_content->horizontalHeader()->setVisible(true);
  this->behavior_content->setSortingEnabled(false);
  this->behavior_content->setContextMenuPolicy(Qt::CustomContextMenu);
  this->behavior_content->viewport()->setFocusPolicy(Qt::NoFocus);
  //connect(behavior_content, SIGNAL(cellClicked(int, int)), this, SLOT(windowFail(std::string title, std::string message)));
  //connect(behavior_content, SIGNAL(cellClicked(int, int)), this, SLOT(cellClickedR(int, int)));
}

void BehaviorExecutionViewer::windowFail(std::string title, std::string message)
{
  QMessageBox *msg_error = new QMessageBox(QMessageBox::Critical, title.c_str(), message.c_str(), QMessageBox::Ok,this);
  msg_error->setWindowFlags(msg_error->windowFlags() | Qt::WindowStaysOnTopHint);
  msg_error->exec();
}

void BehaviorExecutionViewer::cellClickedR(int nRow, int nCol)
{
  QString message = "Behavior activated: " + ((this->behavior_content->item(nRow, 1))->data(Qt::DisplayRole).toString());
  if(((this->behavior_content->item(nRow, 1))->data(Qt::DisplayRole).toString()) == " "){
    QMessageBox *msg_info = new QMessageBox(QMessageBox::Critical, "Execution Viewer Details", "Haz click en un casilla disponible", QMessageBox::Ok,this);
    msg_info->setWindowFlags(msg_info->windowFlags() | Qt::WindowStaysOnTopHint);
    msg_info->hide();
  }
  else{
    if(((this->behavior_content->item(nRow, 2))->data(Qt::DisplayRole).toString()) == ""){
      message += " with no parameters";
    }
    else{
      message += " with parameters: " + ((this->behavior_content->item(nRow, 2))->data(Qt::DisplayRole).toString());
    }

    QMessageBox *msg_info = new QMessageBox(QMessageBox::Information, "Execution Viewer Details", message, QMessageBox::Ok,this);
    msg_info->setWindowFlags(msg_info->windowFlags() | Qt::WindowStaysOnTopHint);
    msg_info->exec();

  }
}


void BehaviorExecutionViewer::behaviorCustomContextMenu(const QPoint& p)
{
  if (this->active_behavior_content->itemAt(p) != 0)
  {
    if (is_behavior_context_menu_created)
    {
      behavior_context_menu->close();
      delete behavior_context_menu;
      is_behavior_context_menu_created = false;
    }
    behavior_context_menu = new QMenu("Menu", this);
    is_behavior_context_menu_created = true;
    this->point = new QPoint(p);

    QAction action_add_behavior("Add a behavior", this->active_behavior_content);
    QAction action_stop_behavior("Stop the execution of this behavior", this->active_behavior_content);

    behavior_context_menu->addAction(&action_add_behavior);
    behavior_context_menu->addAction(&action_stop_behavior);

    connect(&action_add_behavior, SIGNAL(triggered()), this, SLOT(addBehavior()));
    connect(&action_stop_behavior, SIGNAL(triggered()), this, SLOT(stopBehavior()));

    behavior_context_menu->exec(QCursor::pos());
  }
  else
  {
    if (is_behavior_context_menu_created)
    {
      behavior_context_menu->close();
      delete behavior_context_menu;
      is_behavior_context_menu_created = false;
    }
    behavior_context_menu = new QMenu("Menu", this);
    is_behavior_context_menu_created = true;
    this->point = 0;

    QAction action_add_behavior("Add a behavior", this->active_behavior_content);

    behavior_context_menu->addAction(&action_add_behavior);

    connect(&action_add_behavior, SIGNAL(triggered()), this, SLOT(addBehavior()));

    behavior_context_menu->exec(QCursor::pos());
  }
}

void BehaviorExecutionViewer::addBehavior()
{
  BehaviorExecutionViewerDialog* dialog = new BehaviorExecutionViewerDialog(this, 0, 0);
  dialog->show();
}

void BehaviorExecutionViewer::stopBehavior()
{
  QTableWidgetItem* item = this->active_behavior_content->itemAt(*point);
  if (item->column() == 0)
  {
    item = this->active_behavior_content->item(item->row(), item->column() + 1);
  }
  if (item->column() == 1)
  {
    item = this->active_behavior_content->item(item->row(), item->column());
  }
  if (item->column() == 2)
  {
    item = this->active_behavior_content->item(item->row(), item->column() - 1);
  }
  
  BehaviorExecutionViewerDialog* dialog = new BehaviorExecutionViewerDialog(this, 2, item);
}

void BehaviorExecutionViewer::clearBehaviorTable()
{
  this->behavior_content->setRowCount(0);
}

void BehaviorExecutionViewer::resizeEvent(QResizeEvent* event)
{
  this->active_behavior_content->setColumnWidth(0, this->behavior_content->width()*0 / 30);
  this->active_behavior_content->setColumnWidth(1, this->behavior_content->width()*15 / 30);
  this->active_behavior_content->setColumnWidth(2, this->behavior_content->width()*15 / 30);

  this->behavior_content->setColumnWidth(0, this->behavior_content->width()*2.5 / 30);
  this->behavior_content->setColumnWidth(1, this->behavior_content->width()*10.5 / 30);
  this->behavior_content->setColumnWidth(2, this->behavior_content->width()*10.5 / 30);
  this->behavior_content->setColumnWidth(3, this->behavior_content->width()*2 / 30);
  this->behavior_content->setColumnWidth(4, this->behavior_content->width()*2 / 30);
  this->behavior_content->setColumnWidth(5, this->behavior_content->width()*2 / 30);

  QWidget::resizeEvent(event);
}

void BehaviorExecutionViewer::closeEvent(QCloseEvent* event)
{
  //window_event_msg.window = aerostack_msgs::WindowEvent::EXECUTION_VIEWER;
  //window_event_msg.event = aerostack_msgs::WindowEvent::CLOSE;
  //window_event_pub.publish(window_event_msg);
}

void BehaviorExecutionViewer::killMe()
{
#ifdef Q_OS_WIN
  enum
  {
    ExitCode = 0
  };
  ::TerminateProcess(::GetCurrentProcess(), ExitCode);
#else
  qint64 pid = QCoreApplication::applicationPid();
  QProcess::startDetached("kill -9 " + QString::number(pid));
#endif  // Q_OS_WIN
}

/*void BehaviorExecutionViewer::windowOpenCallback(const aerostack_msgs::WindowEvent& msg)
{
 
  if ( msg.window == aerostack_msgs::WindowEvent::ALPHANUMERIC_INTERFACE_CONTROL )
  {
     window_event_msg.window = aerostack_msgs::WindowEvent::EXECUTION_VIEWER;
     window_event_msg.event = aerostack_msgs::WindowEvent::CLOSE;
     window_event_pub.publish(window_event_msg);
     killMe();
  }

  if (msg.window == aerostack_msgs::WindowEvent::INTEGRATED_VIEWER && msg.event == aerostack_msgs::WindowEvent::MINIMIZE)
    showMinimized();
}*/
