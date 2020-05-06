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
#include "../include/autoCam_pkg/saveeditview.hpp"
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

//    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    if ( !qnode.init() ) {
      showNoMasterMessage();
    }

    ui.graphicsView->setScene(new QGraphicsScene(this));
    ui.graphicsView->scene()->addItem(&pixmap);

    ui.graphicsView_2->setScene(new QGraphicsScene(this));
    ui.graphicsView_2->scene()->addItem(&pixmap_2);

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

void MainWindow::on_SetPoint_SaveViewButton_clicked()
{
    SaveEditView saveeditview;
    saveeditview.setModal(true);
    saveeditview.exec();
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

  makeLarger->move(30,80);
  makeLarger->resize(900,675);
  makeSmaller->resize(180,135);
  makeSmaller->move(750,80);
  makeSmaller->raise();

  //increment the counter
  switchCounter ++;

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


}  // namespace autoCam_pkg
