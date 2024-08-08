/**
 * @file /include/robot_one/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robot_one_QNODE_HPP_
#define robot_one_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <boost/thread.hpp>
#include <time.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_one {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
  void set_cmd_vel(char k,float linear,float angular);
  void set_goal(double x,double y,double z,double w);
  void get_points_nav_info(std::vector<geometry_msgs::Pose> pose_list,int times,int stop_time,bool next_state);
  void pub_points_nav_start_flag(bool flag);
  void run_points_nav(bool flag);
  void set_points_nav_next_btn_click();
  void set_stop_points_nav(bool flag);

	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();
  void speed_vel(float,float);
  void position(double x,double y,double z,double w);
  void back_position(double x,double y,double z,double w);
  void points_nav_position(double x,double y,double z,double w);
private:
	int init_argc;
	char** init_argv;
  ros::Publisher cmd_vel_pub;
  ros::Publisher marker_pub;
  ros::Subscriber single_goal_sub;
  ros::Subscriber back_pose_sub;
  ros::Subscriber points_goal_sub;
  ros::Publisher goal_pub;
  QStringListModel logging_model;
  void single_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void back_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void points_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  //巡航设计
  std::vector<geometry_msgs::Pose> pose_list;
  int  points_nav_times;
  int  points_nav_stop_time;
  bool points_nav_next_state;
  bool points_nav_next_btn_click = false;
  int  run_count_now = 0;
  ros::Publisher points_nav_pub;
  ros::Subscriber points_nav_sub;
  boost::thread* run_points_nav_thread_;
  boost::mutex run_points_nav_mutex_;
  void run_points_nav_callback(const std_msgs::Bool::ConstPtr& msg);
  bool stop_points_nav_flag = false; //默认不暂停巡航任务

};

}  // namespace robot_one

#endif /* robot_one_QNODE_HPP_ */
