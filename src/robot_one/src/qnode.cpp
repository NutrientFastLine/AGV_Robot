/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

/***********************************************************************************************************
  ROS人机交互软件,'RobotOne'
（1）新建地图、保存地图、编辑地图
（2）设置单点导航、多点巡航、保存设置内容
（3）类rviz设计，可视化操作
（4）键盘控制节点
（5）自定义功能、自定义单点导航名字
***********************************************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/robot_one/qnode.hpp"
#include <QDebug>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_one {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"robot_one");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  cmd_vel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  marker_pub =n.advertise<visualization_msgs::Marker>("marker", 100);
  single_goal_sub=n.subscribe("single_goal", 100, &QNode::single_goal_callback,this);
  back_pose_sub  =n.subscribe("back_pose", 100, &QNode::back_pose_callback,this);
  points_goal_sub=n.subscribe("points_goal", 100, &QNode::points_goal_callback,this);
  goal_pub=n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",100);
  points_nav_pub = n.advertise<std_msgs::Bool>("points_nav_start_flag", 10);
  points_nav_sub = n.subscribe("points_nav_start_flag", 10, &QNode::run_points_nav_callback,this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"robot_one");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  cmd_vel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  marker_pub =n.advertise<visualization_msgs::Marker>("marker", 100);
  single_goal_sub=n.subscribe("single_goal", 100, &QNode::single_goal_callback,this);
  back_pose_sub  =n.subscribe("back_pose", 100, &QNode::back_pose_callback,this);
  points_goal_sub=n.subscribe("points_goal", 100, &QNode::points_goal_callback,this);
  goal_pub=n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",100);
  points_nav_pub = n.advertise<std_msgs::Bool>("points_nav_start_flag", 10);
  points_nav_sub = n.subscribe("points_nav_start_flag", 10, &QNode::run_points_nav_callback,this);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
  bool sendOnceFlag = false;
  while ( ros::ok() ) {

    if(!sendOnceFlag)
    {
      sendOnceFlag = true;
      log(Info,std::string("Connect Succeed!!!"));
    }
		ros::spinOnce();
    loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

//打印log相关
void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

//键盘控制-设置速度相关
void QNode::set_cmd_vel(char k,float linear,float angular)
{
    // Map for movement keys
    std::map<char, std::vector<float>> moveBindings
    {
      {'i', {1, 0, 0, 0}},
      {'o', {1, 0, 0, -1}},
      {'j', {0, 0, 0, 1}},
      {'l', {0, 0, 0, -1}},
      {'u', {1, 0, 0, 1}},
      {',', {-1, 0, 0, 0}},
      {'.', {-1, 0, 0, 1}},
      {'m', {-1, 0, 0, -1}},
      {'O', {1, -1, 0, 0}},
      {'I', {1, 0, 0, 0}},
      {'J', {0, 1, 0, 0}},
      {'L', {0, -1, 0, 0}},
      {'U', {1, 1, 0, 0}},
      {'<', {-1, 0, 0, 0}},
      {'>', {-1, -1, 0, 0}},
      {'M', {-1, 1, 0, 0}},
      {'t', {0, 0, 1, 0}},
      {'b', {0, 0, -1, 0}},
      {'k', {0, 0, 0, 0}},
      {'K', {0, 0, 0, 0}}
    };
    char key=k;
    // Grab the direction data
    int x = moveBindings[key][0];
    int y = moveBindings[key][1];
    int z = moveBindings[key][2];
    int th = moveBindings[key][3];

    geometry_msgs::Twist twist;
    twist.linear.x=x*linear;
    twist.linear.y=y*linear;
    twist.linear.z=z*linear;

    twist.angular.x=0;
    twist.angular.y=0;
    twist.angular.z=th*angular;

    cmd_vel_pub.publish(twist);
}

//发布单个导航点
void QNode::set_goal(double x,double y,double z,double w)
{
    geometry_msgs::PoseStamped goal;
    //设置frame
    goal.header.frame_id="map";
    //设置时刻
    goal.header.stamp=ros::Time::now();
    goal.pose.position.x=x;
    goal.pose.position.y=y;
    goal.pose.orientation.z=z;
    goal.pose.orientation.w=w;
    //发布导航点
    goal_pub.publish(goal);
}

//获得多点巡航的目标信息
void QNode::get_points_nav_info(std::vector<geometry_msgs::Pose> points_list,int times,int stop_time,bool next_state)
{
  run_points_nav_mutex_.lock();
  pose_list = points_list;

//  qDebug()<<"pose_list"<<pose_list.size();
//  qDebug()<<"Test"<<pose_list[0].position.x<<pose_list[0].position.y;

  points_nav_times = times;
  points_nav_stop_time = stop_time;
  points_nav_next_state = next_state;

  run_points_nav_mutex_.unlock();
//  qDebug()<<"points_nav_times"<<points_nav_times;
//  qDebug()<<"points_nav_stop_time"<<points_nav_stop_time;
//  qDebug()<<"points_nav_next_state"<<points_nav_next_state;
}

//发布多点导航开始标志
void QNode::pub_points_nav_start_flag(bool flag)
{
  std_msgs::Bool sendBool;
  sendBool.data = flag;
  points_nav_pub.publish(sendBool);
}

//多点巡航-下一站按钮状态
void QNode::set_points_nav_next_btn_click()
{
  run_points_nav_mutex_.lock();
  points_nav_next_btn_click = true;
  run_points_nav_mutex_.unlock();
}

//停止多点巡航
void QNode::set_stop_points_nav(bool flag)
{
  run_points_nav_mutex_.lock();
  stop_points_nav_flag = flag;
  run_points_nav_mutex_.unlock();
}

//多点巡航线程任务
void QNode::run_points_nav(bool flag)
{
  ros::Time lastTime,nowTime;

  if(flag)
  {
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> nav_client("move_base",true);
    log(Info,std::string("Waiting for move_base action server..."));
    if(!nav_client.waitForServer(ros::Duration(60))) log(Info,std::string("Can't connected to move base server"));

    log(Info,std::string("Connected to move base server"));
    log(Info,std::string("Starting navigation"));

    //多次循环
    for (int i =0 ; i<points_nav_times ; i++) {
      //循环一次所发布的多个导航点
      while(run_count_now < pose_list.size())
      {
        run_points_nav_mutex_.lock();

        static bool run_once_flag = false;
        if(!run_once_flag)
        {
          move_base_msgs::MoveBaseGoal goal;
          goal.target_pose.header.frame_id = "map";
          goal.target_pose.header.stamp = ros::Time::now();
          goal.target_pose.pose = pose_list[run_count_now];
          nav_client.sendGoal(goal);
          //60S内如果不能达到目标点则放弃该目标点
          bool finished_within_time = nav_client.waitForResult(ros::Duration(60));
          if(!finished_within_time)
          {
            nav_client.cancelGoal();
            log(Info,std::string("Timed out achieving goal"));
          }
          else
          {
            //导航成功
            if(nav_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
              log(Info,std::string("This goal succeeded!"));
            }
            //导航失败
            else
            {
              log(Info,std::string("This goal failed!"));
            }
          }
          run_once_flag = true;
          lastTime = nowTime = ros::Time::now();
          //qDebug()<<"ros::Time::now()"<<nowTime.toSec();
        }

        //默认不暂停巡航任务
        if(!stop_points_nav_flag)
        {
          //没选择-默认自动下一个节点
          if(!points_nav_next_state)
          {
            if(nowTime.toSec() - lastTime.toSec() < points_nav_stop_time)
            {
              nowTime = ros::Time::now();
            }
            else
            {
              run_count_now++;
              run_once_flag = false;
            }
          }
          else //选择-手动下一个节点
          {
            if(points_nav_next_btn_click)
            {
              run_count_now++;
              points_nav_next_btn_click = false;
              run_once_flag = false;
            }
          }
        }

        run_points_nav_mutex_.unlock();
      }
      run_count_now = 0;
    }
    log(Info,std::string("The Cycle Goals is over"));
    pose_list.clear();
  }
}

//开辟多点巡航的线程
void QNode::run_points_nav_callback(const std_msgs::Bool::ConstPtr& msg)
{
//  qDebug() << "msg->data"<<msg->data;
  run_points_nav_thread_ = new boost::thread(boost::bind(&QNode::run_points_nav, this, msg->data));
}

//相关marker
visualization_msgs::Marker arrow;
visualization_msgs::Marker number;
void QNode::single_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    static int num =0;
    //send signal
    emit position(msg->pose.position.x,msg->pose.position.y,msg->pose.orientation.z,msg->pose.orientation.w);

    arrow.header.frame_id =number.header.frame_id= "map";
    arrow.lifetime = number.lifetime= ros::Duration();
    arrow.header.stamp = number.header.stamp=ros::Time::now();
    arrow.ns  = "single_arrows";
    number.ns = "single_numbers";
    //同一个命名空间 id 要不同才会追加
    arrow.id = num;
    number.id = num;
    arrow.type = arrow.ARROW;
    number.type = number.TEXT_VIEW_FACING;
    arrow.action = number.action= visualization_msgs::Marker::ADD;

    arrow.pose = number.pose = msg->pose;
    number.pose.position.z = 1.0;
    arrow.scale.x = 0.8;
    arrow.scale.y = 0.1;
    number.scale.z = 0.5;
    arrow.color.r = number.color.r = 0.00f;
    arrow.color.g = number.color.g = 0.90f;
    arrow.color.b = number.color.b = 0.00f;
    arrow.color.a = number.color.a = 0.8f;

    number.text = std::to_string(num++);

    marker_pub.publish(arrow);
    marker_pub.publish(number);
    //qDebug()<<"test marker pub";
}

void QNode::back_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //send back pose
    emit back_position(msg->pose.position.x,msg->pose.position.y,msg->pose.orientation.z,msg->pose.orientation.w);
    arrow.header.frame_id =number.header.frame_id= "map";
    arrow.lifetime = number.lifetime= ros::Duration();
    arrow.header.stamp = number.header.stamp=ros::Time::now();
    arrow.ns  = "back_arrows";
    number.ns = "back_numbers";
    //同一个命名空间 id 要不同才会追加
    arrow.id = 0;
    number.id = 0;
    arrow.type = arrow.ARROW;
    number.type = number.TEXT_VIEW_FACING;
    arrow.action = number.action= visualization_msgs::Marker::ADD;

    arrow.pose = number.pose = msg->pose;
    number.pose.position.z = 1.0;
    arrow.scale.x = 1.0;
    arrow.scale.y = 0.1;
    number.scale.z = 0.35;
    arrow.color.r = number.color.r = 0.00f;
    arrow.color.g = number.color.g = 0.00f;
    arrow.color.b = number.color.b = 0.80f;
    arrow.color.a = number.color.a = 0.8f;

    number.text = "Return Pose";

    marker_pub.publish(arrow);
    marker_pub.publish(number);
}

visualization_msgs::Marker lineStrip;
void QNode::points_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static int num =0;
  //send points_nav
  emit points_nav_position(msg->pose.position.x,msg->pose.position.y,msg->pose.orientation.z,msg->pose.orientation.w);
  //
  geometry_msgs::Point tempPoint;
  tempPoint.x=msg->pose.position.x;
  tempPoint.y=msg->pose.position.y;
  tempPoint.z=msg->pose.position.z;

  arrow.header.frame_id =number.header.frame_id=lineStrip.header.frame_id= "map";
  arrow.lifetime = number.lifetime= lineStrip.lifetime= ros::Duration();
  arrow.header.stamp = number.header.stamp= lineStrip.header.stamp=ros::Time::now();
  arrow.ns  = "points_arrows";
  number.ns = "points_numbers";
  lineStrip.ns = "points_lineStrips";
  //同一个命名空间 id 要不同才会追加
  arrow.id = num;
  number.id = num;
  lineStrip.id = num;
  arrow.type = arrow.ARROW;
  number.type = number.TEXT_VIEW_FACING;
  lineStrip.type=lineStrip.LINE_STRIP;
  arrow.action = number.action = lineStrip.action= visualization_msgs::Marker::ADD;

  arrow.pose = number.pose = msg->pose;
  lineStrip.points.push_back(tempPoint);
  number.pose.position.z = 1.0;
  arrow.scale.x = 0.8;
  arrow.scale.y = 0.1;
  number.scale.z = 0.5;
  lineStrip.scale.x = 0.08;

  arrow.color.r = number.color.r =lineStrip.color.r = 0.90f;
  arrow.color.g = number.color.g =lineStrip.color.g = 0.00f;
  arrow.color.b = number.color.b =lineStrip.color.b = 0.00f;
  arrow.color.a = number.color.a =lineStrip.color.a = 0.8f;

  number.text = std::to_string(num++);

  marker_pub.publish(arrow);
  marker_pub.publish(number);
  marker_pub.publish(lineStrip);

  //qDebug()<<"test marker pub";
}

}  // namespace robot_one
