/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "ui_main_window.h"
#include "../include/autoCam_pkg/main_window.hpp"
#include "../include/autoCam_pkg/common.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace autoCam_pkg {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  /* Set the window icon because why not */
  setWindowIcon(QIcon(":/images/icon.png"));

  /* Connect to the ROS Master */
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /* Show a warning of the ROS master is not running */
    if ( !qnode.init() ) {
      showNoMasterMessage();
    }

  /* Attach scene and pixmap to the graphics views for the cameras. */
    ui.graphicsView->setScene(new QGraphicsScene(this));
    ui.graphicsView->scene()->addItem(&pixmap);

    ui.graphicsView_2->setScene(new QGraphicsScene(this));
    ui.graphicsView_2->scene()->addItem(&pixmap_2);

  /* Set ragne of joint sliders */
    QSlider *J1_horizontalSlider = new QSlider(Qt::Horizontal);
    J1_horizontalSlider->setRange(-180, 180);
    QSlider *J2_horizontalSlider = new QSlider(Qt::Horizontal);
    J2_horizontalSlider->setRange(-128, 128);
    QSlider *J3_horizontalSlider = new QSlider(Qt::Horizontal);
    J3_horizontalSlider->setRange(-180, 180);
    QSlider *J4_horizontalSlider = new QSlider(Qt::Horizontal);
    J4_horizontalSlider->setRange(-147, 147);
    QSlider *J5_horizontalSlider = new QSlider(Qt::Horizontal);
    J5_horizontalSlider->setRange(-180, 180);
    QSlider *J6_horizontalSlider = new QSlider(Qt::Horizontal);
    J6_horizontalSlider->setRange(-120, 120);
    QSlider *J7_horizontalSlider = new QSlider(Qt::Horizontal);
    J7_horizontalSlider->setRange(-180, 180);
  /*////*/

}



MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
  msgBox.setText("Couldn't find the ROS master.");
	msgBox.exec();
    close();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  imageStream = false;
  //WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::on_Manual_SwitchButton_pressed()
{
  ui.Manual_SwitchButton->setText(QString("Switch Cameras"));

  //figure out which widget should be made larger or smaller
  QGraphicsView* makeLarger = ui.graphicsView;
  QGraphicsView* makeSmaller = ui.graphicsView_2;
  ui.statusBar->showMessage("Showing Head Camera",3500);

  if (switchCounter%2 == 1)
  {
      makeLarger = ui.graphicsView_2;
      makeSmaller = ui.graphicsView;
      ui.statusBar->showMessage("Showing Arm Camera",3500);
  }

  makeLarger->move(5,60);
  makeLarger->resize(640,640);
  makeSmaller->resize(180,180);
  makeSmaller->move(465,60);
  makeSmaller->raise();

  //increment the counter
  switchCounter = switchCounter + 1;
  std::cout << "switching camera \n";


  //check that both image streams are populated with images
  if(my_global_cv_ptr != NULL && my_global_cv_ptr_2 != NULL){
    imageStream = true;
//    std::cout << "Display: image not null \n";
    while(imageStream) {
      if(!my_global_cv_ptr->image.empty()) {
        QImage qimg(my_global_cv_ptr->image.data,
                    my_global_cv_ptr->image.cols,
                    my_global_cv_ptr->image.rows,
                    my_global_cv_ptr->image.step,
                    QImage::Format_RGB888);
        pixmap.setPixmap( QPixmap::fromImage(qimg.rgbSwapped()) );
        ui.graphicsView->fitInView(&pixmap, Qt::KeepAspectRatio);
      }
      if(!my_global_cv_ptr_2->image.empty()) {
        QImage qimg_2(my_global_cv_ptr_2->image.data,
                    my_global_cv_ptr_2->image.cols,
                    my_global_cv_ptr_2->image.rows,
                    my_global_cv_ptr_2->image.step,
                    QImage::Format_RGB888);
        pixmap_2.setPixmap( QPixmap::fromImage(qimg_2.rgbSwapped()) );
        ui.graphicsView_2->fitInView(&pixmap_2, Qt::KeepAspectRatio);
      qApp->processEvents();
      }
    }
  }
}

// 2)
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/

/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
/*///////////////////////////////////////////////////////    Command Window Tab    //////////////////////////////////////////////////////////////*/
/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

/*/////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
// Command Window
// Manual Cartesian Control
// PushButton Names: UP_pushButton, DOWN_pushButton, LEFT_pushButton, RIGHT_pushButton
//                   IN_pushButton, OUT_pushButton
// TextEdit Box Names: X_textEdit, Y_textEdit, Z_textEdit
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_UP_pushButton_clicked() {
    Zcounter = Zcounter + 1.0;
}

void MainWindow::on_DOWN_pushButton_clicked() {
    Zcounter = Zcounter - 1.0;
}

void MainWindow::on_OUT_pushButton_clicked() {
    Xcounter = Xcounter + 1.0;
}

void MainWindow::on_IN_pushButton_clicked() {
    Xcounter = Xcounter - 1.0;
}

void MainWindow::on_RIGHT_pushButton_clicked() {
    Ycounter = Ycounter + 1.0;
}

void MainWindow::on_LEFT_pushButton_clicked() {
    Ycounter = Ycounter - 1.0;
}

void MainWindow::observeCartesian(){

  //update values
  Xcounter = hand_camera_pose.position.x;
  Ycounter = hand_camera_pose.position.y;
  Zcounter = hand_camera_pose.position.z;


  //displays the values in the local variables
  QString strX = QString::number(Xcounter);
  ui.Y_lineEdit->setText(strX);
  QString strY = QString::number(Ycounter);
  ui.Y_lineEdit->setText(strY);
  QString strZ = QString::number(Zcounter);
  ui.Y_lineEdit->setText(strZ);
}

/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Command Window
// Manual Joint Pose Control
// Horizontal Slider Names: J1_horizontalSlider, J2_horizontalSlider, J3_horizontalSlider,
//                          J4_horizontalSlider, J5_horizontalSlider, J6_horizontalSlider,
//                          J7_horizontalSlider
// TextEdit Box Names: J1_textEdit, J2_textEdit, J3_textEdit,J4_textEdit, J5_textEdit,
//                     J6_textEdit, J7_textEdit
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/

void MainWindow::on_J1_horizontalSlider_valueChanged(int value)
{
    //int intValue = value;
    QString b = QString::number(value);
    ui.J1_textEdit->setText(b);
}

void MainWindow::on_J2_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.J2_textEdit->setText(b);
}

void MainWindow::on_J3_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.J3_textEdit->setText(b);
}

void MainWindow::on_J4_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.J4_textEdit->setText(b);
}

void MainWindow::on_J5_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.J5_textEdit->setText(b);
}

void MainWindow::on_J6_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.J6_textEdit->setText(b);
}

void MainWindow::on_J7_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.J7_textEdit->setText(b);
}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Command Window
// Manual Camera Control
// Dial and Slider Names: Roll_dial, Pitch_horizontalSlider, Yaw_verticalSlider
// TextEdit Box Names: Roll_textEdit, Pitch_textEdit, Yaw_textEdit
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_Roll_dial_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.Roll_textEdit->setText(b);
}

void MainWindow::on_Yaw_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.Yaw_textEdit->setText(b);
}

void MainWindow::on_Pitch_verticalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.Pitch_textEdit->setText(b);
}
/*/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
// Command Window
// Standard Views Available
// Push Button Name: StandardView1_pushButton, StandardView2_pushButton, StandardView3_pushButton
//                   StandardView4_pushButton, StandardView5_pushButton
///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////*/


/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Command Window
// User Added View Selection
// Push Button Names: view1_pushButton, view2_pushButton, view3_pushButton, view4_pushButton,
//                    view5_pushButton
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/

void MainWindow::on_view1_pushButton_clicked()
{

}

void MainWindow::on_view2_pushButton_clicked()
{

}

void MainWindow::on_view3_pushButton_clicked()
{

}

void MainWindow::on_view4_pushButton_clicked()
{

}

void MainWindow::on_view5_pushButton_clicked()
{

}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Command Window
// Add a View
// Push Button Name: addView_pushButton
// TextEdit Box Names:addName_textEdit
// Push Button Names: view1_pushButton, view2_pushButton, view3_pushButton, view4_pushButton,
//                    view5_pushButton
// Combo Box Name: view_ComboBox
// This function will replace the empty views untill all are full
// When all are full, you can replace specific views using the comboBox
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/

int count=0;
void MainWindow::on_addView_pushButton_clicked()
{
        count++;
        if (count==1){
          //This view has a name
          QString name1 = ui.addName_textEdit->toPlainText();
          ui.view1_pushButton->setText("1: " + name1);
          ui.addName_textEdit->clear();
          //This view has a cartesian pose for x,yz
          QString X1 = ui.X_lineEdit->text();
          QString Y1 = ui.Y_lineEdit->text();
          QString Z1 = ui.Z_lineEdit->text();
          ui.X_lineEdit->clear();
          ui.Y_lineEdit->clear();
          ui.Z_lineEdit->clear();
          //This view has 7DOF joint angles
          QString J1_1 = ui.J1_textEdit->toPlainText();
          QString J2_1 = ui.J2_textEdit->toPlainText();
          QString J3_1 = ui.J3_textEdit->toPlainText();
          QString J4_1 = ui.J4_textEdit->toPlainText();
          QString J5_1 = ui.J5_textEdit->toPlainText();
          QString J6_1 = ui.J6_textEdit->toPlainText();
          QString J7_1 = ui.J7_textEdit->toPlainText();
          ui.J1_horizontalSlider->setValue(0);
          ui.J2_horizontalSlider->setValue(0);
          ui.J3_horizontalSlider->setValue(0);
          ui.J4_horizontalSlider->setValue(0);
          ui.J5_horizontalSlider->setValue(0);
          ui.J6_horizontalSlider->setValue(0);
          ui.J7_horizontalSlider->setValue(0);
          ui.J1_textEdit->clear();
          ui.J2_textEdit->clear();
          ui.J3_textEdit->clear();
          ui.J4_textEdit->clear();
          ui.J5_textEdit->clear();
          ui.J6_textEdit->clear();
          ui.J7_textEdit->clear();
          //This view has a roll,pitch, and yaw
          QString ROLL1 = ui.Roll_textEdit->toPlainText();
          QString PITCH1 = ui.Pitch_textEdit->toPlainText();
          QString YAW1 = ui.Yaw_textEdit->toPlainText();
          ui.Roll_dial->setValue(0);
          ui.Yaw_horizontalSlider->setValue(0);
          ui.Pitch_verticalSlider->setValue(0);
          ui.Roll_textEdit->clear();
          ui.Pitch_textEdit->clear();
          ui.Yaw_textEdit->clear();
          //Assign these views to View 1
          ui.NEWNAME1->setText(name1);
          ui.NewX1->setText(X1);
          ui.NewY1->setText(Y1);
          ui.NewZ1->setText(Z1);
          ui.NewJ1_1->setText(J1_1);
          ui.NewJ2_1->setText(J2_1);
          ui.NewJ3_1->setText(J3_1);
          ui.NewJ4_1->setText(J4_1);
          ui.NewJ5_1->setText(J5_1);
          ui.NewJ6_1->setText(J6_1);
          ui.NewJ7_1->setText(J7_1);
          ui.NewROLL1->setText(ROLL1);
          ui.NewPITCH1->setText(PITCH1);
          ui.NewYAW1->setText(YAW1);
        }
        if (count==2){
          //This view has a name
          QString name2 = ui.addName_textEdit->toPlainText();
          ui.view2_pushButton->setText("2: " + name2);
          ui.addName_textEdit->clear();
          //This view has a cartesian pose for x,yz
          QString X2 = ui.X_lineEdit->text();
          QString Y2 = ui.Y_lineEdit->text();
          QString Z2 = ui.Z_lineEdit->text();
          ui.X_lineEdit->clear();
          ui.Y_lineEdit->clear();
          ui.Z_lineEdit->clear();
          //This view has 7DOF joint angles
          QString J1_2 = ui.J1_textEdit->toPlainText();
          QString J2_2 = ui.J2_textEdit->toPlainText();
          QString J3_2 = ui.J3_textEdit->toPlainText();
          QString J4_2 = ui.J4_textEdit->toPlainText();
          QString J5_2 = ui.J5_textEdit->toPlainText();
          QString J6_2 = ui.J6_textEdit->toPlainText();
          QString J7_2 = ui.J7_textEdit->toPlainText();
          ui.J1_horizontalSlider->setValue(0);
          ui.J2_horizontalSlider->setValue(0);
          ui.J3_horizontalSlider->setValue(0);
          ui.J4_horizontalSlider->setValue(0);
          ui.J5_horizontalSlider->setValue(0);
          ui.J6_horizontalSlider->setValue(0);
          ui.J7_horizontalSlider->setValue(0);
          ui.J1_textEdit->clear();
          ui.J2_textEdit->clear();
          ui.J3_textEdit->clear();
          ui.J4_textEdit->clear();
          ui.J5_textEdit->clear();
          ui.J6_textEdit->clear();
          ui.J7_textEdit->clear();
          //This view has a roll,pitch, and yaw
          QString ROLL2 = ui.Roll_textEdit->toPlainText();
          QString PITCH2 = ui.Pitch_textEdit->toPlainText();
          QString YAW2 = ui.Yaw_textEdit->toPlainText();
          ui.Roll_dial->setValue(0);
          ui.Yaw_horizontalSlider->setValue(0);
          ui.Pitch_verticalSlider->setValue(0);
          ui.Roll_textEdit->clear();
          ui.Pitch_textEdit->clear();
          ui.Yaw_textEdit->clear();
          //Assign these views to View 2
          ui.NEWNAME2->setText(name2);
          ui.NewX2->setText(X2);
          ui.NewY2->setText(Y2);
          ui.NewZ2->setText(Z2);
          ui.NewJ1_2->setText(J1_2);
          ui.NewJ2_2->setText(J2_2);
          ui.NewJ3_2->setText(J3_2);
          ui.NewJ4_2->setText(J4_2);
          ui.NewJ5_2->setText(J5_2);
          ui.NewJ6_2->setText(J6_2);
          ui.NewJ7_2->setText(J7_2);
          ui.NewROLL2->setText(ROLL2);
          ui.NewPITCH2->setText(PITCH2);
          ui.NewYAW2->setText(YAW2);
        }
        if (count==3){
          //This view has a name
          QString name3 = ui.addName_textEdit->toPlainText();
          ui.view3_pushButton->setText("3: " + name3);
          ui.addName_textEdit->clear();
          //This view has a cartesian pose for x,yz
          QString X3 = ui.X_lineEdit->text();
          QString Y3 = ui.Y_lineEdit->text();
          QString Z3 = ui.Z_lineEdit->text();
          ui.X_lineEdit->clear();
          ui.Y_lineEdit->clear();
          ui.Z_lineEdit->clear();
          //This view has 7DOF joint angles
          QString J1_3 = ui.J1_textEdit->toPlainText();
          QString J2_3 = ui.J2_textEdit->toPlainText();
          QString J3_3 = ui.J3_textEdit->toPlainText();
          QString J4_3 = ui.J4_textEdit->toPlainText();
          QString J5_3 = ui.J5_textEdit->toPlainText();
          QString J6_3 = ui.J6_textEdit->toPlainText();
          QString J7_3 = ui.J7_textEdit->toPlainText();
          ui.J1_horizontalSlider->setValue(0);
          ui.J2_horizontalSlider->setValue(0);
          ui.J3_horizontalSlider->setValue(0);
          ui.J4_horizontalSlider->setValue(0);
          ui.J5_horizontalSlider->setValue(0);
          ui.J6_horizontalSlider->setValue(0);
          ui.J7_horizontalSlider->setValue(0);
          ui.J1_textEdit->clear();
          ui.J2_textEdit->clear();
          ui.J3_textEdit->clear();
          ui.J4_textEdit->clear();
          ui.J5_textEdit->clear();
          ui.J6_textEdit->clear();
          ui.J7_textEdit->clear();
          //This view has a roll,pitch, and yaw
          QString ROLL3 = ui.Roll_textEdit->toPlainText();
          QString PITCH3 = ui.Pitch_textEdit->toPlainText();
          QString YAW3 = ui.Yaw_textEdit->toPlainText();
          ui.Roll_dial->setValue(0);
          ui.Yaw_horizontalSlider->setValue(0);
          ui.Pitch_verticalSlider->setValue(0);
          ui.Roll_textEdit->clear();
          ui.Pitch_textEdit->clear();
          ui.Yaw_textEdit->clear();
          //Assign these views to View 3
          ui.NEWNAME3->setText(name3);
          ui.NewX3->setText(X3);
          ui.NewY3->setText(Y3);
          ui.NewZ3->setText(Z3);
          ui.NewJ1_3->setText(J1_3);
          ui.NewJ2_3->setText(J2_3);
          ui.NewJ3_3->setText(J3_3);
          ui.NewJ4_3->setText(J4_3);
          ui.NewJ5_3->setText(J5_3);
          ui.NewJ6_3->setText(J6_3);
          ui.NewJ7_3->setText(J7_3);
          ui.NewROLL3->setText(ROLL3);
          ui.NewPITCH3->setText(PITCH3);
          ui.NewYAW3->setText(YAW3);
        }
        if (count==4){
          //This view has a name
          QString name4 = ui.addName_textEdit->toPlainText();
          ui.view4_pushButton->setText("4: " + name4);
          ui.addName_textEdit->clear();
          //This view has a cartesian pose for x,yz
          QString X4 = ui.X_lineEdit->text();
          QString Y4 = ui.Y_lineEdit->text();
          QString Z4 = ui.Z_lineEdit->text();
          ui.X_lineEdit->clear();
          ui.Y_lineEdit->clear();
          ui.Z_lineEdit->clear();
          //This view has 7DOF joint angles
          QString J1_4 = ui.J1_textEdit->toPlainText();
          QString J2_4 = ui.J2_textEdit->toPlainText();
          QString J3_4 = ui.J3_textEdit->toPlainText();
          QString J4_4 = ui.J4_textEdit->toPlainText();
          QString J5_4 = ui.J5_textEdit->toPlainText();
          QString J6_4 = ui.J6_textEdit->toPlainText();
          QString J7_4 = ui.J7_textEdit->toPlainText();
          ui.J1_horizontalSlider->setValue(0);
          ui.J2_horizontalSlider->setValue(0);
          ui.J3_horizontalSlider->setValue(0);
          ui.J4_horizontalSlider->setValue(0);
          ui.J5_horizontalSlider->setValue(0);
          ui.J6_horizontalSlider->setValue(0);
          ui.J7_horizontalSlider->setValue(0);
          ui.J1_textEdit->clear();
          ui.J2_textEdit->clear();
          ui.J3_textEdit->clear();
          ui.J4_textEdit->clear();
          ui.J5_textEdit->clear();
          ui.J6_textEdit->clear();
          ui.J7_textEdit->clear();
          //This view has a roll,pitch, and yaw
          QString ROLL4 = ui.Roll_textEdit->toPlainText();
          QString PITCH4 = ui.Pitch_textEdit->toPlainText();
          QString YAW4 = ui.Yaw_textEdit->toPlainText();
          ui.Roll_dial->setValue(0);
          ui.Yaw_horizontalSlider->setValue(0);
          ui.Pitch_verticalSlider->setValue(0);
          ui.Roll_textEdit->clear();
          ui.Pitch_textEdit->clear();
          ui.Yaw_textEdit->clear();
          //Assign these views to View 4
          ui.NEWNAME4->setText(name4);
          ui.NewX4->setText(X4);
          ui.NewY4->setText(Y4);
          ui.NewZ4->setText(Z4);
          ui.NewJ1_4->setText(J1_4);
          ui.NewJ2_4->setText(J2_4);
          ui.NewJ3_4->setText(J3_4);
          ui.NewJ4_4->setText(J4_4);
          ui.NewJ5_4->setText(J5_4);
          ui.NewJ6_4->setText(J6_4);
          ui.NewJ7_4->setText(J7_4);
          ui.NewROLL4->setText(ROLL4);
          ui.NewPITCH4->setText(PITCH4);
          ui.NewYAW4->setText(YAW4);
        }
        if (count==5){
          //This view has a name
          QString name5 = ui.addName_textEdit->toPlainText();
          ui.view5_pushButton->setText("5: " + name5);
          ui.addName_textEdit->clear();
          //This view has a cartesian pose for x,yz
          QString X5 = ui.X_lineEdit->text();
          QString Y5 = ui.Y_lineEdit->text();
          QString Z5 = ui.Z_lineEdit->text();
          ui.X_lineEdit->clear();
          ui.Y_lineEdit->clear();
          ui.Z_lineEdit->clear();
          //This view has 7DOF joint angles
          QString J1_5 = ui.J1_textEdit->toPlainText();
          QString J2_5 = ui.J2_textEdit->toPlainText();
          QString J3_5 = ui.J3_textEdit->toPlainText();
          QString J4_5 = ui.J4_textEdit->toPlainText();
          QString J5_5 = ui.J5_textEdit->toPlainText();
          QString J6_5 = ui.J6_textEdit->toPlainText();
          QString J7_5 = ui.J7_textEdit->toPlainText();
          ui.J1_horizontalSlider->setValue(0);
          ui.J2_horizontalSlider->setValue(0);
          ui.J3_horizontalSlider->setValue(0);
          ui.J4_horizontalSlider->setValue(0);
          ui.J5_horizontalSlider->setValue(0);
          ui.J6_horizontalSlider->setValue(0);
          ui.J7_horizontalSlider->setValue(0);
          ui.J1_textEdit->clear();
          ui.J2_textEdit->clear();
          ui.J3_textEdit->clear();
          ui.J4_textEdit->clear();
          ui.J5_textEdit->clear();
          ui.J6_textEdit->clear();
          ui.J7_textEdit->clear();
          //This view has a roll,pitch, and yaw
          QString ROLL5 = ui.Roll_textEdit->toPlainText();
          QString PITCH5 = ui.Pitch_textEdit->toPlainText();
          QString YAW5 = ui.Yaw_textEdit->toPlainText();
          ui.Roll_dial->setValue(0);
          ui.Yaw_horizontalSlider->setValue(0);
          ui.Pitch_verticalSlider->setValue(0);
          ui.Roll_textEdit->clear();
          ui.Pitch_textEdit->clear();
          ui.Yaw_textEdit->clear();
          //Assign these views to View 5
          ui.NEWNAME5->setText(name5);
          ui.NewX5->setText(X5);
          ui.NewY5->setText(Y5);
          ui.NewZ5->setText(Z5);
          ui.NewJ1_5->setText(J1_5);
          ui.NewJ2_5->setText(J2_5);
          ui.NewJ3_5->setText(J3_5);
          ui.NewJ4_5->setText(J4_5);
          ui.NewJ5_5->setText(J5_5);
          ui.NewJ6_5->setText(J6_5);
          ui.NewJ7_5->setText(J7_5);
          ui.NewROLL5->setText(ROLL5);
          ui.NewPITCH5->setText(PITCH5);
          ui.NewYAW5->setText(YAW5);
        }
        /*else {
            QString currentState = ui.view_comboBox->currentText();
            if (currentState == "View 1"){
                QString name1 = ui.addName_textEdit->toPlainText();
                ui.view1_pushButton->setText("1: " + name1);
                ui.addName_textEdit->clear();}
            if (currentState == "View 2"){
                QString name2 = ui.addName_textEdit->toPlainText();
                ui.view2_pushButton->setText("2: " + name2);
                ui.addName_textEdit->clear();}
            if (currentState == "View 3"){
                QString name3 = ui.addName_textEdit->toPlainText();
                ui.view3_pushButton->setText("3: " + name3);
                ui.addName_textEdit->clear();}
            if (currentState == "View 4"){
                QString name4 = ui.addName_textEdit->toPlainText();
                ui.view4_pushButton->setText("4: " + name4);
                ui.addName_textEdit->clear();}
            if (currentState == "View 5"){
                QString name5 = ui.addName_textEdit->toPlainText();
                ui.view5_pushButton->setText("5: " + name5);
                ui.addName_textEdit->clear();
            }
        }*/
}

/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
/*///////////////////////////////////////////////////////    View Editor Tab    /////////////////////////////////////////////////////////////////*/
/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

/*/////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
// Edit Views Window
// Edit Cartesian Control
// PushButton Names: NewUP_pushButton, NewDOWN_pushButton, NewLEFT_pushButton,
//                   NewRIGHT_pushButton, NewIN_pushButton, NewOUT_pushButton
// TextEdit Box Names: XNEW_textEdit, YNEW_textEdit, ZNEW_textEdit
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_NewUP_pushButton_clicked()
{
    ZNEWcounter++;
    QString strZ = QString::number(ZNEWcounter);
    ui.ZNEW_textEdit->setText(strZ);
}

void MainWindow::on_NewDOWN_pushButton_clicked()
{
    ZNEWcounter--;
    QString strZ = QString::number(ZNEWcounter);
    ui.ZNEW_textEdit->setText(strZ);
}

void MainWindow::on_NewLEFT_pushButton_clicked()
{
    YNEWcounter--;
    QString strY = QString::number(YNEWcounter);
    ui.YNEW_textEdit->setText(strY);
}

void MainWindow::on_NewRIGHT_pushButton_clicked()
{
    YNEWcounter++;
    QString strY = QString::number(YNEWcounter);
    ui.YNEW_textEdit->setText(strY);
}

void MainWindow::on_NewIN_pushButton_clicked()
{
    XNEWcounter--;
    QString strX = QString::number(XNEWcounter);
    ui.XNEW_textEdit->setText(strX);
}

void MainWindow::on_NewOUT_pushButton_clicked()
{
    XNEWcounter++;
    QString strX = QString::number(XNEWcounter);
    ui.XNEW_textEdit->setText(strX);
}
/*//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Edit Views Window
// Edit Joint Pose Control
// Horizontal Slider Names: NewJ1_horizontalSlider, NewJ2_horizontalSlider, NewJ3_horizontalSlider,
//                          NewJ4_horizontalSlider, NewJ5_horizontalSlider, NewJ6_horizontalSlider,
//                          NewJ7_horizontalSlider
// TextEdit Box Names: NewJ1_textEdit, NewJ2_textEdit, NewJ3_textEdit, NewJ4_textEdit,
//                     NewJ5_textEdit, NewJ6_textEdit, NewJ7_textEdit
////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_NewJ1_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ1_textEdit->setText(b);
}

void MainWindow::on_NewJ2_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ2_textEdit->setText(b);
}

void MainWindow::on_NewJ3_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ3_textEdit->setText(b);
}

void MainWindow::on_NewJ4_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ4_textEdit->setText(b);
}

void MainWindow::on_NewJ5_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ5_textEdit->setText(b);
}

void MainWindow::on_NewJ6_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ6_textEdit->setText(b);
}

void MainWindow::on_NewJ7_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ7_textEdit->setText(b);
}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Edit Views Window
// Edit Camera Control
// Dial and Slider Names: NewRoll_dial, NewPitch_horizontalSlider, NewYaw_verticalSlider
// TextEdit Box Names: NewRoll_textEdit, NewPitch_textEdit, NewYaw_textEdit
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/

void MainWindow::on_Newdial_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewRoll_textEdit->setText(b);
}

void MainWindow::on_New_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewYaw_textEdit->setText(b);
}

void MainWindow::on_New_verticalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewPitch_textEdit->setText(b);
}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Edit Views Window
// Change View Name
// Push Button Name: applyNewName_pushButton
// TextEdit Box Names:newName_textEdit
// Push Button Names: view1_pushButton, view2_pushButton, view3_pushButton, view4_pushButton,
//                    view5_pushButton
// Combo Box Name: selectView_comboBox
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_applyNewName_pushButton_clicked()
{
    QString currentState = ui.selectView_comboBox->currentText();
    if (currentState == "View 1"){
        QString name = ui.newName_textEdit->toPlainText();
        ui.view1_pushButton->setText("1: " + name);
        ui.newName_textEdit->clear();
        ui.NEWNAME1->setText(name);
    }
    if (currentState == "View 2"){
        QString name = ui.newName_textEdit->toPlainText();
        ui.view2_pushButton->setText("2: " + name);
        ui.newName_textEdit->clear();
        ui.NEWNAME2->setText(name);
    }
    if (currentState == "View 3"){
        QString name = ui.newName_textEdit->toPlainText();
        ui.view3_pushButton->setText("3: " + name);
        ui.newName_textEdit->clear();
    }
    if (currentState == "View 4"){
        QString name = ui.newName_textEdit->toPlainText();
        ui.view4_pushButton->setText("4: " + name);
        ui.newName_textEdit->clear();
    }
    if (currentState == "View 5"){
        QString name = ui.newName_textEdit->toPlainText();
        ui.view5_pushButton->setText("5: " + name);
        ui.newName_textEdit->clear();
    }
}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Edit Views Window
// Apply All Changes
// Push Button Name:
// TextEdit Box Names:
// Push Button Names:
// Combo Box Name:
//This function clears and saves all new information when Apply All Changes is clicked
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_ApplyAllChanges_pushButton_clicked()
{
    QString currentState = ui.selectView_comboBox->currentText();
    if (currentState == "View 1"){
        QString name = ui.newName_textEdit->toPlainText();
        ui.view1_pushButton->setText("1: " + name);
        ui.NEWNAME1->setText(name);
        //ui.newName_textEdit->clear();

        QString NewX1 = ui.XNEW_textEdit->toPlainText();
        QString NewY1 = ui.YNEW_textEdit->toPlainText();
        QString NewZ1 = ui.ZNEW_textEdit->toPlainText();
        ui.NewX1->setText(NewX1);
        ui.NewY1->setText(NewY1);
        ui.NewZ1->setText(NewZ1);
        /*ui.XNEW_textEdit->clear();
        ui.YNEW_textEdit->clear();
        ui.ZNEW_textEdit->clear();*/

        QString NewJ1_1 = ui.NewJ1_textEdit->toPlainText();
        QString NewJ2_1 = ui.NewJ2_textEdit->toPlainText();
        QString NewJ3_1 = ui.NewJ3_textEdit->toPlainText();
        QString NewJ4_1 = ui.NewJ4_textEdit->toPlainText();
        QString NewJ5_1 = ui.NewJ5_textEdit->toPlainText();
        QString NewJ6_1 = ui.NewJ6_textEdit->toPlainText();
        QString NewJ7_1 = ui.NewJ7_textEdit->toPlainText();
        ui.NewJ1_1->setText(NewJ1_1);
        ui.NewJ2_1->setText(NewJ2_1);
        ui.NewJ3_1->setText(NewJ3_1);
        ui.NewJ4_1->setText(NewJ4_1);
        ui.NewJ5_1->setText(NewJ5_1);
        ui.NewJ6_1->setText(NewJ6_1);
        ui.NewJ7_1->setText(NewJ7_1);
        /*ui.NewJ1_textEdit->clear();
        ui.NewJ2_textEdit->clear();
        ui.NewJ3_textEdit->clear();
        ui.NewJ4_textEdit->clear();
        ui.NewJ5_textEdit->clear();
        ui.NewJ6_textEdit->clear();
        ui.NewJ7_textEdit->clear();*/

        QString NewROLL1 = ui.NewRoll_textEdit->toPlainText();
        QString NewPITCH1 = ui.NewPitch_textEdit->toPlainText();
        QString NewYAW1 = ui.NewYaw_textEdit->toPlainText();
        ui.NewROLL1->setText(NewROLL1);
        ui.NewPITCH1->setText(NewPITCH1);
        ui.NewYAW1->setText(NewYAW1);
        /*ui.NewRoll_textEdit->clear();
        ui.NewPitch_textEdit->clear();
        ui.NewYaw_textEdit->clear();*/

       /* ui.NewJ1_horizontalSlider->setValue(0);
        ui.NewJ2_horizontalSlider->setValue(0);
        ui.NewJ3_horizontalSlider->setValue(0);
        ui.NewJ4_horizontalSlider->setValue(0);
        ui.NewJ5_horizontalSlider->setValue(0);
        ui.NewJ6_horizontalSlider->setValue(0);
        ui.NewJ7_horizontalSlider->setValue(0);*/
    }
    if (currentState == "View 2"){
        QString name = ui.newName_textEdit->toPlainText();
        ui.view2_pushButton->setText("2: " + name);
        ui.NEWNAME2->setText(name);
        //ui.newName_textEdit->clear();

        QString NewX2 = ui.XNEW_textEdit->toPlainText();
        QString NewY2 = ui.YNEW_textEdit->toPlainText();
        QString NewZ2 = ui.ZNEW_textEdit->toPlainText();
        ui.NewX2->setText(NewX2);
        ui.NewY2->setText(NewY2);
        ui.NewZ2->setText(NewZ2);
        /*ui.XNEW_textEdit->clear();
        ui.YNEW_textEdit->clear();
        ui.ZNEW_textEdit->clear();*/

        QString NewJ1_2 = ui.NewJ1_textEdit->toPlainText();
        QString NewJ2_2 = ui.NewJ2_textEdit->toPlainText();
        QString NewJ3_2 = ui.NewJ3_textEdit->toPlainText();
        QString NewJ4_2 = ui.NewJ4_textEdit->toPlainText();
        QString NewJ5_2 = ui.NewJ5_textEdit->toPlainText();
        QString NewJ6_2 = ui.NewJ6_textEdit->toPlainText();
        QString NewJ7_2 = ui.NewJ7_textEdit->toPlainText();
        ui.NewJ1_2->setText(NewJ1_2);
        ui.NewJ2_2->setText(NewJ2_2);
        ui.NewJ3_2->setText(NewJ3_2);
        ui.NewJ4_2->setText(NewJ4_2);
        ui.NewJ5_2->setText(NewJ5_2);
        ui.NewJ6_2->setText(NewJ6_2);
        ui.NewJ7_2->setText(NewJ7_2);
        /*ui.NewJ1_textEdit->clear();
        ui.NewJ2_textEdit->clear();
        ui.NewJ3_textEdit->clear();
        ui.NewJ4_textEdit->clear();
        ui.NewJ5_textEdit->clear();
        ui.NewJ6_textEdit->clear();
        ui.NewJ7_textEdit->clear();*/

        QString NewROLL2 = ui.NewRoll_textEdit->toPlainText();
        QString NewPITCH2 = ui.NewPitch_textEdit->toPlainText();
        QString NewYAW2 = ui.NewYaw_textEdit->toPlainText();
        ui.NewROLL2->setText(NewROLL2);
        ui.NewPITCH2->setText(NewPITCH2);
        ui.NewYAW2->setText(NewYAW2);
        /*ui.NewRoll_textEdit->clear();
        ui.NewPitch_textEdit->clear();
        ui.NewYaw_textEdit->clear();*/
    }
    if (currentState == "View 3"){
        QString name = ui.newName_textEdit->toPlainText();
        ui.view3_pushButton->setText("3: " + name);
        //ui.newName_textEdit->clear();
        ui.NEWNAME3->setText(name);

        QString NewX3 = ui.XNEW_textEdit->toPlainText();
        QString NewY3 = ui.YNEW_textEdit->toPlainText();
        QString NewZ3 = ui.ZNEW_textEdit->toPlainText();
        ui.NewX3->setText(NewX3);
        ui.NewY3->setText(NewY3);
        ui.NewZ3->setText(NewZ3);
        /*ui.XNEW_textEdit->clear();
        ui.YNEW_textEdit->clear();
        ui.ZNEW_textEdit->clear();*/

        QString NewJ1_3 = ui.NewJ1_textEdit->toPlainText();
        QString NewJ2_3 = ui.NewJ2_textEdit->toPlainText();
        QString NewJ3_3 = ui.NewJ3_textEdit->toPlainText();
        QString NewJ4_3 = ui.NewJ4_textEdit->toPlainText();
        QString NewJ5_3 = ui.NewJ5_textEdit->toPlainText();
        QString NewJ6_3 = ui.NewJ6_textEdit->toPlainText();
        QString NewJ7_3 = ui.NewJ7_textEdit->toPlainText();
        ui.NewJ1_3->setText(NewJ1_3);
        ui.NewJ2_3->setText(NewJ2_3);
        ui.NewJ3_3->setText(NewJ3_3);
        ui.NewJ4_3->setText(NewJ4_3);
        ui.NewJ5_3->setText(NewJ5_3);
        ui.NewJ6_3->setText(NewJ6_3);
        ui.NewJ7_3->setText(NewJ7_3);
        /*ui.NewJ1_textEdit->clear();
        ui.NewJ2_textEdit->clear();
        ui.NewJ3_textEdit->clear();
        ui.NewJ4_textEdit->clear();
        ui.NewJ5_textEdit->clear();
        ui.NewJ6_textEdit->clear();
        ui.NewJ7_textEdit->clear();*/

        QString NewROLL3 = ui.NewRoll_textEdit->toPlainText();
        QString NewPITCH3 = ui.NewPitch_textEdit->toPlainText();
        QString NewYAW3 = ui.NewYaw_textEdit->toPlainText();
        ui.NewROLL3->setText(NewROLL3);
        ui.NewPITCH3->setText(NewPITCH3);
        ui.NewYAW3->setText(NewYAW3);
        /*ui.NewRoll_textEdit->clear();
        ui.NewPitch_textEdit->clear();
        ui.NewYaw_textEdit->clear();*/
    }
    if (currentState == "View 4"){
        QString name = ui.newName_textEdit->toPlainText();
        ui.view4_pushButton->setText("4: " + name);
        //ui.newName_textEdit->clear();
        ui.NEWNAME4->setText(name);

        QString NewX4 = ui.XNEW_textEdit->toPlainText();
        QString NewY4 = ui.YNEW_textEdit->toPlainText();
        QString NewZ4 = ui.ZNEW_textEdit->toPlainText();
        ui.NewX4->setText(NewX4);
        ui.NewY4->setText(NewY4);
        ui.NewZ4->setText(NewZ4);
        /*ui.XNEW_textEdit->clear();
        ui.YNEW_textEdit->clear();
        ui.ZNEW_textEdit->clear();*/

        QString NewJ1_4 = ui.NewJ1_textEdit->toPlainText();
        QString NewJ2_4 = ui.NewJ2_textEdit->toPlainText();
        QString NewJ3_4 = ui.NewJ3_textEdit->toPlainText();
        QString NewJ4_4 = ui.NewJ4_textEdit->toPlainText();
        QString NewJ5_4 = ui.NewJ5_textEdit->toPlainText();
        QString NewJ6_4 = ui.NewJ6_textEdit->toPlainText();
        QString NewJ7_4 = ui.NewJ7_textEdit->toPlainText();
        ui.NewJ1_4->setText(NewJ1_4);
        ui.NewJ2_4->setText(NewJ2_4);
        ui.NewJ3_4->setText(NewJ3_4);
        ui.NewJ4_4->setText(NewJ4_4);
        ui.NewJ5_4->setText(NewJ5_4);
        ui.NewJ6_4->setText(NewJ6_4);
        ui.NewJ7_4->setText(NewJ7_4);
        /*ui.NewJ1_textEdit->clear();
        ui.NewJ2_textEdit->clear();
        ui.NewJ3_textEdit->clear();
        ui.NewJ4_textEdit->clear();
        ui.NewJ5_textEdit->clear();
        ui.NewJ6_textEdit->clear();
        ui.NewJ7_textEdit->clear();*/

        QString NewROLL4 = ui.NewRoll_textEdit->toPlainText();
        QString NewPITCH4 = ui.NewPitch_textEdit->toPlainText();
        QString NewYAW4 = ui.NewYaw_textEdit->toPlainText();
        ui.NewROLL4->setText(NewROLL4);
        ui.NewPITCH4->setText(NewPITCH4);
        ui.NewYAW4->setText(NewYAW4);
        /*ui.NewRoll_textEdit->clear();
        ui.NewPitch_textEdit->clear();
        ui.NewYaw_textEdit->clear();*/
    }
    if (currentState == "View 5"){
        QString name = ui.newName_textEdit->toPlainText();
        ui.view5_pushButton->setText("5: " + name);
        //ui.newName_textEdit->clear();
        ui.NEWNAME5->setText(name);

        QString NewX5 = ui.XNEW_textEdit->toPlainText();
        QString NewY5 = ui.YNEW_textEdit->toPlainText();
        QString NewZ5 = ui.ZNEW_textEdit->toPlainText();
        ui.NewX5->setText(NewX5);
        ui.NewY5->setText(NewY5);
        ui.NewZ5->setText(NewZ5);
        /*ui.XNEW_textEdit->clear();
        ui.YNEW_textEdit->clear();
        ui.ZNEW_textEdit->clear();*/

        QString NewJ1_5 = ui.NewJ1_textEdit->toPlainText();
        QString NewJ2_5 = ui.NewJ2_textEdit->toPlainText();
        QString NewJ3_5 = ui.NewJ3_textEdit->toPlainText();
        QString NewJ4_5 = ui.NewJ4_textEdit->toPlainText();
        QString NewJ5_5 = ui.NewJ5_textEdit->toPlainText();
        QString NewJ6_5 = ui.NewJ6_textEdit->toPlainText();
        QString NewJ7_5 = ui.NewJ7_textEdit->toPlainText();
        ui.NewJ1_5->setText(NewJ1_5);
        ui.NewJ2_5->setText(NewJ2_5);
        ui.NewJ3_5->setText(NewJ3_5);
        ui.NewJ4_5->setText(NewJ4_5);
        ui.NewJ5_5->setText(NewJ5_5);
        ui.NewJ6_5->setText(NewJ6_5);
        ui.NewJ7_5->setText(NewJ7_5);
        /*ui.NewJ1_textEdit->clear();
        ui.NewJ2_textEdit->clear();
        ui.NewJ3_textEdit->clear();
        ui.NewJ4_textEdit->clear();
        ui.NewJ5_textEdit->clear();
        ui.NewJ6_textEdit->clear();
        ui.NewJ7_textEdit->clear();*/

        QString NewROLL5 = ui.NewRoll_textEdit->toPlainText();
        QString NewPITCH5 = ui.NewPitch_textEdit->toPlainText();
        QString NewYAW5 = ui.NewYaw_textEdit->toPlainText();
        ui.NewROLL5->setText(NewROLL5);
        ui.NewPITCH5->setText(NewPITCH5);
        ui.NewYAW5->setText(NewYAW5);
        /*ui.NewRoll_textEdit->clear();
        ui.NewPitch_textEdit->clear();
        ui.NewYaw_textEdit->clear();*/
    }
}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Edit Views Window
// Edit Specific View
// Push Button Name:
// TextEdit Box Names:
// Push Button Names:
// Combo Box Name:
// This function populates the tab with the information specific to that tab
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_selectView_comboBox_activated(const QString &arg1)
{
    if(arg1 == "View 1"){
        QString NEWNAME1 = ui.NEWNAME1->toPlainText();
        QString NewX1 = ui.NewX1->toPlainText();
        QString NewY1 = ui.NewY1->toPlainText();
        QString NewZ1 = ui.NewZ1->toPlainText();
        QString NewJ1_1 = ui.NewJ1_1->toPlainText();
        QString NewJ2_1 = ui.NewJ2_1->toPlainText();
        QString NewJ3_1 = ui.NewJ3_1->toPlainText();
        QString NewJ4_1 = ui.NewJ4_1->toPlainText();
        QString NewJ5_1 = ui.NewJ5_1->toPlainText();
        QString NewJ6_1 = ui.NewJ6_1->toPlainText();
        QString NewJ7_1 = ui.NewJ7_1->toPlainText();
        QString NewROLL1 = ui.NewROLL1->toPlainText();
        QString NewPITCH1 = ui.NewPITCH1->toPlainText();
        QString NewYAW1 = ui.NewYAW1->toPlainText();
        ui.newName_textEdit->setText(NEWNAME1);
        ui.XNEW_textEdit->setText(NewX1);
        ui.YNEW_textEdit->setText(NewY1);
        ui.ZNEW_textEdit->setText(NewZ1);
        ui.NewJ1_textEdit->setText(NewJ1_1);
        float value1 = NewJ1_1.toFloat();
        ui.NewJ1_horizontalSlider->setValue(value1);
        ui.NewJ2_textEdit->setText(NewJ2_1);
        float value2 = NewJ2_1.toFloat();
        ui.NewJ2_horizontalSlider->setValue(value2);
        ui.NewJ3_textEdit->setText(NewJ3_1);
        float value3 = NewJ3_1.toFloat();
        ui.NewJ3_horizontalSlider->setValue(value3);
        ui.NewJ4_textEdit->setText(NewJ4_1);
        float value4 = NewJ4_1.toFloat();
        ui.NewJ4_horizontalSlider->setValue(value4);
        ui.NewJ5_textEdit->setText(NewJ5_1);
        float value5 = NewJ5_1.toFloat();
        ui.NewJ5_horizontalSlider->setValue(value5);
        ui.NewJ6_textEdit->setText(NewJ6_1);
        float value6 = NewJ6_1.toFloat();
        ui.NewJ6_horizontalSlider->setValue(value6);
        ui.NewJ7_textEdit->setText(NewJ7_1);
        float value7 = NewJ7_1.toFloat();
        ui.NewJ7_horizontalSlider->setValue(value7);
        ui.NewRoll_textEdit->setText(NewROLL1);
        ui.NewPitch_textEdit->setText(NewPITCH1);
        ui.NewYaw_textEdit->setText(NewYAW1);
    }
    else if(arg1 == "View 2"){
        QString NEWNAME2 = ui.NEWNAME2->toPlainText();
        ui.newName_textEdit->setText(NEWNAME2);
        QString NewX2 = ui.NewX2->toPlainText();
        QString NewY2 = ui.NewY2->toPlainText();
        QString NewZ2 = ui.NewZ2->toPlainText();
        QString NewJ1_2 = ui.NewJ1_2->toPlainText();
        QString NewJ2_2 = ui.NewJ2_2->toPlainText();
        QString NewJ3_2 = ui.NewJ3_2->toPlainText();
        QString NewJ4_2 = ui.NewJ4_2->toPlainText();
        QString NewJ5_2 = ui.NewJ5_2->toPlainText();
        QString NewJ6_2 = ui.NewJ6_2->toPlainText();
        QString NewJ7_2 = ui.NewJ7_2->toPlainText();
        QString NewROLL2 = ui.NewROLL2->toPlainText();
        QString NewPITCH2 = ui.NewPITCH2->toPlainText();
        QString NewYAW2 = ui.NewYAW2->toPlainText();
        ui.newName_textEdit->setText(NEWNAME2);
        ui.XNEW_textEdit->setText(NewX2);
        ui.YNEW_textEdit->setText(NewY2);
        ui.ZNEW_textEdit->setText(NewZ2);
        ui.NewJ1_textEdit->setText(NewJ1_2);
        float value1 = NewJ1_2.toFloat();
        ui.NewJ1_horizontalSlider->setValue(value1);
        ui.NewJ2_textEdit->setText(NewJ2_2);
        float value2 = NewJ2_2.toFloat();
        ui.NewJ2_horizontalSlider->setValue(value2);
        ui.NewJ3_textEdit->setText(NewJ3_2);
        float value3 = NewJ3_2.toFloat();
        ui.NewJ3_horizontalSlider->setValue(value3);
        ui.NewJ4_textEdit->setText(NewJ4_2);
        float value4 = NewJ4_2.toFloat();
        ui.NewJ4_horizontalSlider->setValue(value4);
        ui.NewJ5_textEdit->setText(NewJ5_2);
        float value5 = NewJ5_2.toFloat();
        ui.NewJ5_horizontalSlider->setValue(value5);
        ui.NewJ6_textEdit->setText(NewJ6_2);
        float value6 = NewJ6_2.toFloat();
        ui.NewJ6_horizontalSlider->setValue(value6);
        ui.NewJ7_textEdit->setText(NewJ7_2);
        float value7 = NewJ7_2.toFloat();
        ui.NewJ7_horizontalSlider->setValue(value7);
        ui.NewRoll_textEdit->setText(NewROLL2);
        ui.NewPitch_textEdit->setText(NewPITCH2);
        ui.NewYaw_textEdit->setText(NewYAW2);

    }
    else if(arg1 == "View 3"){
        QString NEWNAME3 = ui.NEWNAME3->toPlainText();
        ui.newName_textEdit->setText(NEWNAME3);
        QString NewX3 = ui.NewX3->toPlainText();
        QString NewY3 = ui.NewY3->toPlainText();
        QString NewZ3 = ui.NewZ3->toPlainText();
        QString NewJ1_3 = ui.NewJ1_3->toPlainText();
        QString NewJ2_3 = ui.NewJ2_3->toPlainText();
        QString NewJ3_3 = ui.NewJ3_3->toPlainText();
        QString NewJ4_3 = ui.NewJ4_3->toPlainText();
        QString NewJ5_3 = ui.NewJ5_3->toPlainText();
        QString NewJ6_3 = ui.NewJ6_3->toPlainText();
        QString NewJ7_3 = ui.NewJ7_3->toPlainText();
        QString NewROLL3 = ui.NewROLL3->toPlainText();
        QString NewPITCH3 = ui.NewPITCH3->toPlainText();
        QString NewYAW3 = ui.NewYAW3->toPlainText();
        ui.newName_textEdit->setText(NEWNAME3);
        ui.XNEW_textEdit->setText(NewX3);
        ui.YNEW_textEdit->setText(NewY3);
        ui.ZNEW_textEdit->setText(NewZ3);
        ui.NewJ1_textEdit->setText(NewJ1_3);
        float value1 = NewJ1_3.toFloat();
        ui.NewJ1_horizontalSlider->setValue(value1);
        ui.NewJ2_textEdit->setText(NewJ2_3);
        float value2 = NewJ2_3.toFloat();
        ui.NewJ2_horizontalSlider->setValue(value2);
        ui.NewJ3_textEdit->setText(NewJ3_3);
        float value3 = NewJ3_3.toFloat();
        ui.NewJ3_horizontalSlider->setValue(value3);
        ui.NewJ4_textEdit->setText(NewJ4_3);
        float value4 = NewJ4_3.toFloat();
        ui.NewJ4_horizontalSlider->setValue(value4);
        ui.NewJ5_textEdit->setText(NewJ5_3);
        float value5 = NewJ5_3.toFloat();
        ui.NewJ5_horizontalSlider->setValue(value5);
        ui.NewJ6_textEdit->setText(NewJ6_3);
        float value6 = NewJ6_3.toFloat();
        ui.NewJ6_horizontalSlider->setValue(value6);
        ui.NewJ7_textEdit->setText(NewJ7_3);
        float value7 = NewJ7_3.toFloat();
        ui.NewJ7_horizontalSlider->setValue(value7);
        ui.NewRoll_textEdit->setText(NewROLL3);
        ui.NewPitch_textEdit->setText(NewPITCH3);
        ui.NewYaw_textEdit->setText(NewYAW3);
    }
    else if(arg1 == "View 4"){
        QString NEWNAME4 = ui.NEWNAME4->toPlainText();
        ui.newName_textEdit->setText(NEWNAME4);
        QString NewX4 = ui.NewX4->toPlainText();
        QString NewY4 = ui.NewY4->toPlainText();
        QString NewZ4 = ui.NewZ4->toPlainText();
        QString NewJ1_4 = ui.NewJ1_4->toPlainText();
        QString NewJ2_4 = ui.NewJ2_4->toPlainText();
        QString NewJ3_4 = ui.NewJ3_4->toPlainText();
        QString NewJ4_4 = ui.NewJ4_4->toPlainText();
        QString NewJ5_4 = ui.NewJ5_4->toPlainText();
        QString NewJ6_4 = ui.NewJ6_4->toPlainText();
        QString NewJ7_4 = ui.NewJ7_4->toPlainText();
        QString NewROLL4 = ui.NewROLL4->toPlainText();
        QString NewPITCH4 = ui.NewPITCH4->toPlainText();
        QString NewYAW4 = ui.NewYAW4->toPlainText();
        ui.newName_textEdit->setText(NEWNAME4);
        ui.XNEW_textEdit->setText(NewX4);
        ui.YNEW_textEdit->setText(NewY4);
        ui.ZNEW_textEdit->setText(NewZ4);
        ui.NewJ1_textEdit->setText(NewJ1_4);
        float value1 = NewJ1_4.toFloat();
        ui.NewJ1_horizontalSlider->setValue(value1);
        ui.NewJ2_textEdit->setText(NewJ2_4);
        float value2 = NewJ2_4.toFloat();
        ui.NewJ2_horizontalSlider->setValue(value2);
        ui.NewJ3_textEdit->setText(NewJ3_4);
        float value3 = NewJ3_4.toFloat();
        ui.NewJ3_horizontalSlider->setValue(value3);
        ui.NewJ4_textEdit->setText(NewJ4_4);
        float value4 = NewJ4_4.toFloat();
        ui.NewJ4_horizontalSlider->setValue(value4);
        ui.NewJ5_textEdit->setText(NewJ5_4);
        float value5 = NewJ5_4.toFloat();
        ui.NewJ5_horizontalSlider->setValue(value5);
        ui.NewJ6_textEdit->setText(NewJ6_4);
        float value6 = NewJ6_4.toFloat();
        ui.NewJ6_horizontalSlider->setValue(value6);
        ui.NewJ7_textEdit->setText(NewJ7_4);
        float value7 = NewJ7_4.toFloat();
        ui.NewJ7_horizontalSlider->setValue(value7);
        ui.NewRoll_textEdit->setText(NewROLL4);
        ui.NewPitch_textEdit->setText(NewPITCH4);
        ui.NewYaw_textEdit->setText(NewYAW4);
    }
    else if (arg1 == "View 5"){
        QString NEWNAME5 = ui.NEWNAME5->toPlainText();
        ui.newName_textEdit->setText(NEWNAME5);
        QString NewX5 = ui.NewX5->toPlainText();
        QString NewY5 = ui.NewY5->toPlainText();
        QString NewZ5 = ui.NewZ5->toPlainText();
        QString NewJ1_5 = ui.NewJ1_5->toPlainText();
        QString NewJ2_5 = ui.NewJ2_5->toPlainText();
        QString NewJ3_5 = ui.NewJ3_5->toPlainText();
        QString NewJ4_5 = ui.NewJ4_5->toPlainText();
        QString NewJ5_5 = ui.NewJ5_5->toPlainText();
        QString NewJ6_5 = ui.NewJ6_5->toPlainText();
        QString NewJ7_5 = ui.NewJ7_5->toPlainText();
        QString NewROLL5 = ui.NewROLL5->toPlainText();
        QString NewPITCH5 = ui.NewPITCH5->toPlainText();
        QString NewYAW5 = ui.NewYAW5->toPlainText();
        ui.newName_textEdit->setText(NEWNAME5);
        ui.XNEW_textEdit->setText(NewX5);
        ui.YNEW_textEdit->setText(NewY5);
        ui.ZNEW_textEdit->setText(NewZ5);
        ui.NewJ1_textEdit->setText(NewJ1_5);
        float value1 = NewJ1_5.toFloat();
        ui.NewJ1_horizontalSlider->setValue(value1);
        ui.NewJ2_textEdit->setText(NewJ2_5);
        float value2 = NewJ2_5.toFloat();
        ui.NewJ2_horizontalSlider->setValue(value2);
        ui.NewJ3_textEdit->setText(NewJ3_5);
        float value3 = NewJ3_5.toFloat();
        ui.NewJ3_horizontalSlider->setValue(value3);
        ui.NewJ4_textEdit->setText(NewJ4_5);
        float value4 = NewJ4_5.toFloat();
        ui.NewJ4_horizontalSlider->setValue(value4);
        ui.NewJ5_textEdit->setText(NewJ5_5);
        float value5 = NewJ5_5.toFloat();
        ui.NewJ5_horizontalSlider->setValue(value5);
        ui.NewJ6_textEdit->setText(NewJ6_5);
        float value6 = NewJ6_5.toFloat();
        ui.NewJ6_horizontalSlider->setValue(value6);
        ui.NewJ7_textEdit->setText(NewJ7_5);
        float value7 = NewJ7_5.toFloat();
        ui.NewJ7_horizontalSlider->setValue(value7);
        ui.NewRoll_textEdit->setText(NewROLL5);
        ui.NewPitch_textEdit->setText(NewPITCH5);
        ui.NewYaw_textEdit->setText(NewYAW5);    }
}
/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
/*///////////////////////////////////////////////////////    Rlexed IK Editor Tab    ////////////////////////////////////////////////////////////*/
/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Relaxed IK Editor Window
// Update Specific Variable
// Dial and Slider Names:
// TextEdit Box Names:
// This function saves specific new variable information
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_SpeedUpdate_pushButton_clicked()
{
    //QString CurrentSpeedVal = ui.SpeedVal_textEdit->toPlainText();
    //ui.SpeedVal_textEdit->setText(CurrentSpeedVal);
    QString CurrentSpeedWeight = ui.SpeedWeight_textEdit->toPlainText();
    ui.SpeedWeight_textEdit->setText(CurrentSpeedWeight);
}

void MainWindow::on_ViewingUpdate_pushButton_clicked()
{
    //QString CurrentViewingVal = ui.ViewingVal_textEdit->toPlainText();
    //ui.ViewingVal_textEdit->setText(CurrentViewingVal);
    QString CurrentViewingWeight = ui.ViewingWeight_textEdit->toPlainText();
    ui.ViewingWeight_textEdit->setText(CurrentViewingWeight);
}

void MainWindow::on_ElecationUpdate_pushButton_clicked()
{
    //QString CurrentElevationVal = ui.ElevationVal_textEdit->toPlainText();
    //ui.ElevationVal_textEdit->setText(CurrentElevationVal);
    QString CurrentElevationWeight = ui.ElevationWeight_textEdit->toPlainText();
    ui.ElevationWeight_textEdit->setText(CurrentElevationWeight);
}

void MainWindow::on_RollUpdate_pushButton_clicked()
{
    QString CurrentRollWeight = ui.RollWeight_textEdit->toPlainText();
    ui.RollWeight_textEdit->setText(CurrentRollWeight);
}

void MainWindow::on_HandUpdate_pushButton_clicked()
{
    QString CurrentHandWeight = ui.HandWeight_textEdit->toPlainText();
    ui.HandWeight_textEdit->setText(CurrentHandWeight);
}

/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Relaxed IK Editor Window
// Update Specific Variable
// Dial and Slider Names:
// TextEdit Box Names:
// This function resets the values and weights
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_ResetVar_pushButton_clicked()
{
    ui.SpeedWeight_textEdit->setText("6");
    ui.Speed_horizontalSlider->setValue(6);

    ui.ViewingWeight_textEdit->setText("50");
    ui.ViewingDistance_horizontalSlider->setValue(50);

    ui.ElevationWeight_textEdit->setText("10");
    ui.Elevation_horizontalSlider->setValue(10);

    ui.RollWeight_textEdit->setText("2");
    ui.CameraRoll_horizontalSlider->setValue(2);

    ui.HandWeight_textEdit->setText("5");
    ui.LookAtHand_horizontalSlider->setValue(5);
}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Relaxed IK Editor Window
// Update Specific Variable
// Dial and Slider Names:
// TextEdit Box Names:
// This function updates and saves the values and weights
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_UpdateAllVars_pushButton_clicked()
{
    //QString CurrentSpeedVal = ui.SpeedVal_textEdit->toPlainText();
    //ui.SpeedVal_textEdit->setText(CurrentSpeedVal);
    QString CurrentSpeedWeight = ui.SpeedWeight_textEdit->toPlainText();
    ui.SpeedWeight_textEdit->setText(CurrentSpeedWeight);
    float value1 = CurrentSpeedWeight.toFloat();
    ui.Speed_horizontalSlider->setValue(value1);

    //QString CurrentViewingVal = ui.ViewingVal_textEdit->toPlainText();
    //ui.ViewingVal_textEdit->setText(CurrentViewingVal);
    QString CurrentViewingWeight = ui.ViewingWeight_textEdit->toPlainText();
    ui.ViewingWeight_textEdit->setText(CurrentViewingWeight);
    float value2 = CurrentViewingWeight.toFloat();
    ui.ViewingDistance_horizontalSlider->setValue(value2);

    //QString CurrentElevationVal = ui.ElevationVal_textEdit->toPlainText();
    //ui.ElevationVal_textEdit->setText(CurrentElevationVal);
    QString CurrentElevationWeight = ui.ElevationWeight_textEdit->toPlainText();
    ui.ElevationWeight_textEdit->setText(CurrentElevationWeight);
    float value3 = CurrentElevationWeight.toFloat();
    ui.Elevation_horizontalSlider->setValue(value3);

    QString CurrentRollWeight = ui.RollWeight_textEdit->toPlainText();
    ui.RollWeight_textEdit->setText(CurrentRollWeight);
    float value4 = CurrentRollWeight.toFloat();
    ui.CameraRoll_horizontalSlider->setValue(value4);


    QString CurrentHandWeight = ui.HandWeight_textEdit->toPlainText();
    ui.HandWeight_textEdit->setText(CurrentHandWeight);
    float value5 = CurrentHandWeight.toFloat();
    ui.LookAtHand_horizontalSlider->setValue(value5);
}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Relaxed IK Editor Window
// Weights Sliders
// Slider Names:
// TextEdit Box Names:
// This function updates the text edit boxes with the weights given by the sliders
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_Speed_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.SpeedWeight_textEdit->setText(b);
}

void MainWindow::on_ViewingDistance_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.ViewingWeight_textEdit->setText(b);
}

void MainWindow::on_Elevation_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.ElevationWeight_textEdit->setText(b);
}

void MainWindow::on_CameraRoll_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.RollWeight_textEdit->setText(b);
}

void MainWindow::on_LookAtHand_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.HandWeight_textEdit->setText(b);
}


void MainWindow::on_X_lineEdit_editingFinished()
{

}

/* Change control method */

void MainWindow::on_comboBox_currentIndexChanged(const QString &arg1)
{
  /* Build the string message */
  std::stringstream ss;
  if (arg1 == "joint"){
    ss << "joint";
    toggleCartesianControl(false);
  }
  else if (arg1 == "cartesian") {
    ss << "cartesian";
    toggleCartesianControl(true);
  }
  else {
    ss << "objective";
    toggleCartesianControl(false);
  }

  /* publish updated control method */
  controlState.data = ss.str();
  qnode.publishControl();
}

/* enables or disables cartesian controls, when disabled they display current state */
void MainWindow::toggleCartesianControl(bool enable) {
  if (enable) {
    //buttons
    ui.LEFT_pushButton->setEnabled(true);
    ui.RIGHT_pushButton->setEnabled(true);
    ui.UP_pushButton->setEnabled(true);
    ui.DOWN_pushButton->setEnabled(true);
    ui.IN_pushButton->setEnabled(true);
    ui.OUT_pushButton->setEnabled(true);
    //text edits
    ui.X_lineEdit->setReadOnly(false);
    ui.Y_lineEdit->setReadOnly(false);
    ui.Z_lineEdit->setReadOnly(false);
  }
  else {
    //buttons
    ui.LEFT_pushButton->setEnabled(false);
    ui.RIGHT_pushButton->setEnabled(false);
    ui.UP_pushButton->setEnabled(false);
    ui.DOWN_pushButton->setEnabled(false);
    ui.IN_pushButton->setEnabled(false);
    ui.OUT_pushButton->setEnabled(false);
    //text edits
    ui.X_lineEdit->setReadOnly(true);
    ui.Y_lineEdit->setReadOnly(true);
    ui.Z_lineEdit->setReadOnly(true);
  }
}


}  // namespace autoCam_pkg
