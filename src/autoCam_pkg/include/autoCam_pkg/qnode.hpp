/**
 * @file /include/autoCam_pkg/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef autoCam_pkg_QNODE_HPP_
#define autoCam_pkg_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829

#include <string>
#include <QThread>
#include <QStringListModel>
#include "common.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace autoCam_pkg {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
  void run();

Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
  image_transport::Subscriber image_sub;
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
};


}  // namespace autoCam_pkg

#endif /* autoCam_pkg_QNODE_HPP_ */
