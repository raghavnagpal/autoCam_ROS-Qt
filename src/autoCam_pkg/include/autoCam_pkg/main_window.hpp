/**
 * @file /include/autoCam_pkg/main_window.hpp
 *
 * @brief Qt based gui for autoCam_pkg.
 *
 * @date November 2010
 **/
#ifndef autoCam_pkg_MAIN_WINDOW_H
#define autoCam_pkg_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
//#include "saveeditview.hpp"
#include "qnode.hpp"
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <QImage>
#include <QGraphicsScene>
#include "common.h"


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace autoCam_pkg {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

//	void ReadSettings(); // Load up qt program settings at startup
//	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
  void on_SetPoint_SaveViewButton_clicked();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
//	void on_actionAbout_triggered();
//	void on_button_connect_clicked(bool check );
//	void on_checkbox_use_environment_stateChanged(int state);
//  void on_Manual_SwitchButton_clicked();

    /******************************************
    ** Manual connections
    *******************************************/
//    void updateLoggingView(); // no idea why this can't connect automatically

private Q_SLOTS:
  void on_Manual_SwitchButton_pressed();

private:
  Ui::MainWindow ui;
	QNode qnode;
  QGraphicsPixmapItem pixmap;
  bool imageStream;
  cv::Mat frame;

};

}  // namespace autoCam_pkg

#endif // autoCam_pkg_MAIN_WINDOW_H
