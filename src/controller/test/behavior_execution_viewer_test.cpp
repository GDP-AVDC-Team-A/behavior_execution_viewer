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
Notas Laura sobre los test

Carpetas que hay que manejar:
-behavior_execution_viewer_process -> todo
-behavior_tree_interpreter_process -> en el fichero behavior_tree_control.cpp está el botón de start mission, se llama execute_tree_button

Cómo hacer click? Mirar la info de QTest::mouseClick en internet
Puedes hacer click con el botón derecho y con el izquierdo (mira en internet QTest)

Test (que se me ocurren) que tiene que haber:
-Test1: Comprobar que se abre la ventana
-Test2: En la tabla de "Execution sequence" comprobar que se rellena. Yo simularía un click en el botón de start mission y comprobaría 
que el número de filas es distinto de 0.
-Test3: En la tabla de "Execution sequence" puedes hacer click con el botón izquierdo y te da info de la fila. Puedes dar click en 
cualquier columna de la tabla, no tiene por qué ser en la primera. Si haces click en un fila en la que la que no hay nada te dice 
que hagas click en otra parte. De toda esta info pueden salir un par de tests:
    -Test3.1: Hacer click en una fila con cosas
    -Test3.2: Hacer click en una fila vacía
-Test4: En la tabla de "Active behaviors" también hay que comprobar que después de darle a start mission se rellena la tabla. Número de 
filas !=0.
-Test5: En la tabla de "Active behaviors" puedes hacer click con el botón derecho y añadir o parar la ejecución de un behavior. No se yo si
esto va a ser demasiado factible, pero al menos habría que comprobar que:
    -Test5.1: Click derecho + añadir behavior, abre una ventana
    -Test5.2: Click derecho + parar ejecución, para la ejecución del behavior.
*/


#include "aerostack_msgs/BehaviorCommand.h"
#include "aerostack_msgs/RequestBehavior.h"
#include "behavior_execution_viewer.h"
#include "../../../../behavior_tree_interpreter_process/src/controller/include/behavior_tree_control.h" //tienes que cambiar este include porque se romperá con la nueva instalación
#include <QApplication>
#include <QMessageBox>
#include <cstdio>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>
#include <QtTest/QtTest>
#include <iostream>

QApplication* app;
BehaviorExecutionViewer* w;
BehaviorTreeControl* tree;
int ar;
bool finished = false;
int total_subtests = 0;
int passed_subtests = 0;

void spinnerThread()
{
  while (!finished)
  {
    ros::spinOnce();
  }
}

void displaySubtestResults()
{
  std::cout << "\033[1;33m TOTAL SUBTESTS: " << total_subtests << "\033[0m" << std::endl;
  std::cout << "\033[1;33m SUBTESTS PASSED: " << passed_subtests << "\033[0m" << std::endl;
  std::cout << "\033[1;33m % PASSED: " << (passed_subtests * 100.0) / (total_subtests * 1.0) << "\033[0m" << std::endl;
}

TEST(BehaviorExecutionViewerTest, editorInitialization) //Se crea bien la ventana
{
    w= new BehaviorExecutionViewer(0, nullptr);
    w->show();
    EXPECT_TRUE(w->getActiveBehaviorExecutionViewer()->rowCount()==0);
    w->~BehaviorExecutionViewer();
}

TEST(BehaviorExecutionViewerTest, addRow) //Se añaden bien las filas a active_behavior_content
{
    w= new BehaviorExecutionViewer(0, nullptr);
    w->show();
    w->getActiveBehaviorExecutionViewer()->insertRow(5);
    w->getActiveBehaviorExecutionViewer()->setItem(0,0,new QTableWidgetItem(QString::fromStdString("Hola2")));
    w->getActiveBehaviorExecutionViewer()->setItem(1,0,new QTableWidgetItem(QString::fromStdString("Hola2")));
    EXPECT_TRUE(w->getActiveBehaviorExecutionViewer()->rowCount()==2);
    w->~BehaviorExecutionViewer();
}

TEST(BehaviorExecutionViewerTest, addRow2) //Se añaden bien las filas a active_behavior_content
{
    w= new BehaviorExecutionViewer(0, nullptr);
    w->show();
    tree = new BehaviorTreeControl();

    QTest::mouseClick(tree->execute_tree_button, Qt::LeftButton, Qt::NoModifier, QPoint(1,2));
    

    EXPECT_TRUE(w->getActiveBehaviorExecutionViewer()->rowCount()!=0);
    w->~BehaviorExecutionViewer();
}
/*
TEST(GraphicalUserInterfaceTests, executionViewerTest)
{
  std::thread thr(&test);
  app->exec();
  thr.join();
}
*/
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  ar = argc;

  system("bash $AEROSTACK_STACK/launchers/launcher_simulated_quadrotor_basic_3.0.sh");
  system(
      "xfce4-terminal  \--tab --title \"Behavior Coordinator\"  --command \"bash -c 'roslaunch behavior_coordinator_process behavior_coordinator_process.launch --wait \
           my_stack_directory:=${AEROSTACK_STACK};exec bash'\" &");
  app = new QApplication(argc, nullptr);
  //w = new BehaviorExecutionViewer(argc, nullptr);
  //w->show();

  std::thread thr(&spinnerThread);

  return RUN_ALL_TESTS();
  thr.join();
}
