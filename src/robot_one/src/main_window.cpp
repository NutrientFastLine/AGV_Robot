/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2021
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

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/robot_one/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_one {

using namespace Qt;

//一些全局变量
QImage *img_robot_connect_state = new QImage;
QImage *img_nav_state_1 = new QImage;
QImage *img_nav_state_2 = new QImage;

//单点导航点存储,12个
std::vector<MyPose> single_nav_goals(12);
//返航点存储
MyPose back_pose;
//多点导航存储
std::vector<MyPose> points_nav_goals;

#define QPIXMAP_SIZE 300
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon("://images/robot.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  //设置主窗口的名字
  this->setWindowTitle("Robot_One - Xidian University!");
  //未开启rosmaster
  roscore_state = false;
  //默认离线图片
  img_robot_connect_state->load("://images/offline.png");
  //设置图片在标签上显示
  ui.label_connect_state->setPixmap(QPixmap::fromImage(*img_robot_connect_state));

  //设置 checkBox next默认状态
  ui.checkBox_nav_next_flag->setCheckState(Checked);

  //默认不激活相关按钮
  setButtonsEnable(false);

  //将导航路径画出来
  drawNavPath();

  /*********************
  ** Auto Start
  **********************/
  if ( ui.checkbox_remember_settings->isChecked() ) {
      on_button_connect_clicked();
  }

  //========================================按键控制部分===================================

  //========================================按键控制部分===================================

  ui.horizontalSlider_linera->setValue(25);
  ui.horizontalSlider_raw->setValue(25);
  ui.label_linera->setText("25");
  ui.label_raw->setText("25");
  //线速度和角速度的进度条
  connect(ui.horizontalSlider_linera,SIGNAL(valueChanged(int)),this,SLOT(slot_linera_value_change(int)));
  connect(ui.horizontalSlider_raw,SIGNAL(valueChanged(int)),this,SLOT(slot_raw_value_change(int)));

  //按下发送控制速度
  connect(ui.pushButton_i, SIGNAL(clicked()),this,SLOT(slot_ctrl_btn_press()));
  connect(ui.pushButton_j, SIGNAL(clicked()),this,SLOT(slot_ctrl_btn_press()));
  connect(ui.pushButton_l, SIGNAL(clicked()),this,SLOT(slot_ctrl_btn_press()));
  connect(ui.pushButton_n, SIGNAL(clicked()),this,SLOT(slot_ctrl_btn_press()));
  connect(ui.pushButton_m, SIGNAL(clicked()),this,SLOT(slot_ctrl_btn_press()));
  connect(ui.pushButton_br,SIGNAL(clicked()),this,SLOT(slot_ctrl_btn_press()));
  connect(ui.pushButton_u, SIGNAL(clicked()),this,SLOT(slot_ctrl_btn_press()));
  connect(ui.pushButton_o, SIGNAL(clicked()),this,SLOT(slot_ctrl_btn_press()));
  connect(ui.pushButton_k, SIGNAL(clicked()),this,SLOT(slot_ctrl_btn_press()));
  //释放发送0


  //=====================================设置按钮图标======================================

  ui.new_map_btn->setIcon(QIcon("://images/classes/Map.png"));
  ui.save_map_btn->setIcon(QIcon("://images/default_package_icon.png"));
  ui.edit_map_btn->setIcon(QIcon("://images/classes/Image.png"));

  ui.initpoint_nav_btn->setIcon(QIcon("://images/classes/SetGoal.png"));
  ui.onepoint_nav_btn->setIcon(QIcon("://images/classes/SetGoal.png"));
  ui.points_nav_btn->setIcon(QIcon("://images/classes/Pose.png"));
  ui.backpoints_nav_btn->setIcon(QIcon("://images/set_return.png"));

  ui.launch_nav_one_btn->setIcon(QIcon("://images/robot2.png"));
  ui.stop_nav_one_btn->setIcon(QIcon("://images/error.png"));
  ui.back_nav_one_btn->setIcon(QIcon("://images/return.png"));

  ui.launch_nav_points_btn->setIcon(QIcon("://images/robot2.png"));
  ui.stop_nav_points_btn->setIcon(QIcon("://images/error.png"));
  ui.next_nav_points_btn->setIcon(QIcon("://images/Navigate.png"));
  ui.back_nav_points_btn->setIcon(QIcon("://images/return.png"));

  //==================================设置单点导航按钮名称===================================

  ui.A01_btn->setText(ui.textEdit_A01->toPlainText());
  ui.A02_btn->setText(ui.textEdit_A02->toPlainText());
  ui.A03_btn->setText(ui.textEdit_A03->toPlainText());
  ui.B01_btn->setText(ui.textEdit_B01->toPlainText());
  ui.B02_btn->setText(ui.textEdit_B02->toPlainText());
  ui.B03_btn->setText(ui.textEdit_B03->toPlainText());
  ui.C01_btn->setText(ui.textEdit_C01->toPlainText());
  ui.C02_btn->setText(ui.textEdit_C02->toPlainText());
  ui.C03_btn->setText(ui.textEdit_C03->toPlainText());
  ui.D01_btn->setText(ui.textEdit_D01->toPlainText());
  ui.D02_btn->setText(ui.textEdit_D02->toPlainText());
  ui.D03_btn->setText(ui.textEdit_D03->toPlainText());

  //导航状态默认图片
  img_nav_state_1->load("://images/forbidden.png");
  ui.label_nav_state_1->setPixmap(QPixmap::fromImage(*img_nav_state_1));
  img_nav_state_2->load("://images/forbidden.png");
  ui.label_nav_state_2->setPixmap(QPixmap::fromImage(*img_nav_state_2));

  //设置开始页面图片
  QImage *img = new QImage;
  img->load("://images/robot_start_2.jpg");
  ui.label_start_img->setScaledContents(true);
  ui.label_start_img->setPixmap(QPixmap::fromImage(*img));

  //设置公众号页面图片
  QImage *img_ad = new QImage;
  img_ad->load("://png/gongzhonghao.png");
  ui.label_ad_img->setScaledContents(true);
  ui.label_ad_img->setPixmap(QPixmap::fromImage(*img_ad));

  //设置treeWidget
  //header
  ui.treeWidget->setHeaderLabels(QStringList()<<"key"<<"value");
  ui.treeWidget->setHeaderHidden(true);

  //add GLobal options
  QTreeWidgetItem* Global=new QTreeWidgetItem(QStringList()<<"Global Options");
  Global->setIcon(0,QIcon("://images/options.png"));
  ui.treeWidget->addTopLevelItem(Global);
  Global->setExpanded(true);

  //FixFrame
  QTreeWidgetItem* Fixed_frame=new QTreeWidgetItem(QStringList()<<"Fixed Frame");
  fixed_box=new QComboBox();
  fixed_box->addItem("map");
  fixed_box->setMaximumWidth(100);
  fixed_box->setEditable(true);
  Global->addChild(Fixed_frame);
  connect(fixed_box,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_value_change(QString)));
  ui.treeWidget->setItemWidget(Fixed_frame,1,fixed_box);

  //add Grid
  QTreeWidgetItem* Grid =new QTreeWidgetItem(QStringList()<<"Grid");
  //设置图标
  Grid->setIcon(0,QIcon("://images/classes/Grid.png"));
  //checkbox
  Grid_Check=new QCheckBox();
  connect(Grid_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_grid(int)));
  //添加top节点
  ui.treeWidget->addTopLevelItem(Grid);
  //添加checkbox
  ui.treeWidget->setItemWidget(Grid,1,Grid_Check);
  //设置grid默认展开状态
  Grid->setExpanded(true);

  //添加Cell Count子节点
  QTreeWidgetItem* Cell_Count=new QTreeWidgetItem(QStringList()<<"Plane Cell Count");
  Grid->addChild(Cell_Count);
  //CellCount添加SpinBox
  Cell_Count_Box=new QSpinBox();
  Cell_Count_Box->setValue(13);
  //设置Spinbox的宽度
  Cell_Count_Box->setMaximumWidth(150);
  ui.treeWidget->setItemWidget(Cell_Count,1,Cell_Count_Box);

  //添加color子节点
  QTreeWidgetItem* Grid_Color=new QTreeWidgetItem(QStringList()<<"Color");
  Grid->addChild(Grid_Color);
  //Color添加ComboBox
  Grid_Color_Box=new QComboBox();
  Grid_Color_Box->addItem("160;160;160");
  //设置Comboox可编辑
  Grid_Color_Box->setEditable(true);
  //设置Combox的宽度
  Grid_Color_Box->setMaximumWidth(150);
  ui.treeWidget->setItemWidget(Grid_Color,1,Grid_Color_Box);

  //TF ui
  QTreeWidgetItem* TF=new QTreeWidgetItem(QStringList()<<"TF");
  //设置图标
  TF->setIcon(0,QIcon("://images/classes/TF.png"));
  //checkbox
  QCheckBox* TF_Check=new QCheckBox();
  connect(TF_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_tf(int)));
  //向Treewidget添加TF Top节点
  ui.treeWidget->addTopLevelItem(TF);
  //向TF添加checkbox
  ui.treeWidget->setItemWidget(TF,1,TF_Check);

  //LaserScan
  QTreeWidgetItem* LaserScan=new QTreeWidgetItem(QStringList()<<"LaserScan");
  //设置图标
  LaserScan->setIcon(0,QIcon("://images/classes/LaserScan.png"));
  //checkbox
  QCheckBox* Laser_Check=new QCheckBox();
  connect(Laser_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_laser(int)));
  //向Treewidget添加TF Top节点
  ui.treeWidget->addTopLevelItem(LaserScan);
  //向TF添加checkbox
  ui.treeWidget->setItemWidget(LaserScan,1,Laser_Check);
  //laser topic
  QTreeWidgetItem* LaserTopic=new QTreeWidgetItem(QStringList()<<"Topic");
  Laser_Topic_box=new QComboBox();
  Laser_Topic_box->addItem("/scan");
  Laser_Topic_box->setEditable(true);
  Laser_Topic_box->setMaximumWidth(150);
  LaserScan->addChild(LaserTopic);
  ui.treeWidget->setItemWidget(LaserTopic,1,Laser_Topic_box);

  //Map
  QTreeWidgetItem* Map=new QTreeWidgetItem(QStringList()<<"Map");
  //设置图标
  Map->setIcon(0,QIcon("://images/classes/Map.png"));
  //checkbox
  Map_Check=new QCheckBox();
  connect(Map_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_Map(int)));
  //向Treewidget添加Map Top节点
  ui.treeWidget->addTopLevelItem(Map);
  //向Map添加checkbox
  ui.treeWidget->setItemWidget(Map,1,Map_Check);
  //Map topic
  QTreeWidgetItem* MapTopic=new QTreeWidgetItem(QStringList()<<"Topic");
  Map_Topic_box=new QComboBox();
  Map_Topic_box->addItem("/map");
  Map_Topic_box->setEditable(true);
  Map_Topic_box->setMaximumWidth(150);
  Map->addChild(MapTopic);
  ui.treeWidget->setItemWidget(MapTopic,1,Map_Topic_box);
  //Map color scheme
  QTreeWidgetItem* MapColorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
  Map_Color_Scheme_box=new QComboBox();
  Map_Color_Scheme_box->addItem("map");
  Map_Color_Scheme_box->addItem("costmap");
  Map_Color_Scheme_box->addItem("raw");
  Map_Color_Scheme_box->setMaximumWidth(150);
  Map->addChild(MapColorScheme);
  ui.treeWidget->setItemWidget(MapColorScheme,1,Map_Color_Scheme_box);

  //Marker
  QTreeWidgetItem* Marker=new QTreeWidgetItem(QStringList()<<"Marker");
  //设置图标
  Marker->setIcon(0,QIcon("://images/classes/Marker.png"));
  //checkbox
  QCheckBox* Marker_Check=new QCheckBox();
  connect(Marker_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_marker(int)));
  //向Treewidget添加Marker Top节点
  ui.treeWidget->addTopLevelItem(Marker);
  //向Marker添加checkbox
  ui.treeWidget->setItemWidget(Marker,1,Marker_Check);
  //Marker topic
  QTreeWidgetItem* MarkerTopic=new QTreeWidgetItem(QStringList()<<"Marker Topic");
  Marker_Topic_box=new QComboBox();
  Marker_Topic_box->addItem("/marker");
  Marker_Topic_box->setEditable(true);
  Marker_Topic_box->setMaximumWidth(150);
  Marker->addChild(MarkerTopic);
  ui.treeWidget->setItemWidget(MarkerTopic,1,Marker_Topic_box);

  //connect
  connect(&qnode,SIGNAL(position(double,double,double,double)),this,SLOT(slot_update_onepoint_nav_pos(double,double,double,double)));
  connect(&qnode,SIGNAL(back_position(double,double,double,double)),this,SLOT(slot_update_back_nav_pos(double,double,double,double)));
  connect(&qnode,SIGNAL(points_nav_position(double,double,double,double)),this,SLOT(slot_update_points_nav_pos(double,double,double,double)));
  //new cmd
  new_map_cmd=new QProcess;
  save_map_cmd=new QProcess;
  launch_onepoint_nav_cmd=new QProcess;
  launch_stop_nav_cmd=new QProcess;
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
  close();
}

//按钮使能
void MainWindow::setButtonsEnable(bool state)
{
  ui.new_map_btn->setEnabled(state);
  ui.save_map_btn->setEnabled(state);
  ui.initpoint_nav_btn->setEnabled(state);
  ui.onepoint_nav_btn->setEnabled(state);
  ui.points_nav_btn->setEnabled(state);
  ui.backpoints_nav_btn->setEnabled(state);
  ui.launch_nav_one_btn->setEnabled(state);
  ui.stop_nav_one_btn->setEnabled(state);
  ui.back_nav_one_btn->setEnabled(state);
  ui.launch_nav_points_btn->setEnabled(state);
  ui.stop_nav_points_btn->setEnabled(state);
  ui.next_nav_points_btn->setEnabled(state);
  ui.back_nav_points_btn->setEnabled(state);

  ui.pushButton_i->setEnabled(state);
  ui.pushButton_j->setEnabled(state);
  ui.pushButton_l->setEnabled(state);
  ui.pushButton_n->setEnabled(state);
  ui.pushButton_m->setEnabled(state);
  ui.pushButton_br->setEnabled(state);
  ui.pushButton_u->setEnabled(state);
  ui.pushButton_o->setEnabled(state);

  ui.A01_btn->setEnabled(state);
  ui.A02_btn->setEnabled(state);
  ui.A03_btn->setEnabled(state);
  ui.B01_btn->setEnabled(state);
  ui.B02_btn->setEnabled(state);
  ui.B03_btn->setEnabled(state);
  ui.C01_btn->setEnabled(state);
  ui.C02_btn->setEnabled(state);
  ui.C03_btn->setEnabled(state);
  ui.D01_btn->setEnabled(state);
  ui.D02_btn->setEnabled(state);
  ui.D03_btn->setEnabled(state);
}


//将多点导航的路径绘制出来
void MainWindow::drawNavPath()
{
  if(points_nav_goals.size())
  {
    //将导航路径画出来
    QSize points_nav_img_size(QPIXMAP_SIZE,QPIXMAP_SIZE);
    QPixmap pix = QPixmap(points_nav_img_size);//以_pixmap作为画布
    pix.fill(Qt::white);
    QPainter p(&pix);//将_pixmap作为画布

    QPen pen;
    pen.setColor(Qt::red);
    pen.setWidth(4);
    p.setRenderHint(QPainter::Antialiasing);//开启抗锯齿
    p.setPen(pen);

    for(unsigned long j=0; j<points_nav_goals.size()-1; ++j)//将线条的所有线段描绘出
    {
       QPoint lineStart(static_cast<int>(pix.width()/2.0 +(points_nav_goals[j].x - points_nav_goals[0].x)*10),
                        static_cast<int>(pix.height()/2.0 -(points_nav_goals[j].y - points_nav_goals[0].y)*10));
       QPoint lineEnd(static_cast<int>(pix.width()/2.0 +(points_nav_goals[j+1].x - points_nav_goals[0].x)*10),
                      static_cast<int>(pix.height()/2.0 -(points_nav_goals[j+1].y - points_nav_goals[0].y)*10));

       if(j==0) p.drawEllipse(lineStart,5,5);
       p.drawLine(lineStart,lineEnd);
    }

    ui.label_points_nav_img->setScaledContents(true);
    ui.label_points_nav_img->setPixmap(pix);
  }
}
/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */


void MainWindow::on_button_connect_clicked() {

  //image online
  img_robot_connect_state->load("://images/online.png");
  if ( ui.checkbox_use_environment->isChecked())
  {
    if ( !qnode.init() )
    {
			showNoMasterMessage();
      ui.treeWidget->setEnabled(false);
      roscore_state = false;
    }
    else
    {
			ui.button_connect->setEnabled(false);
      ui.treeWidget->setEnabled(true);
      myrviz=new QRviz(ui.layout_rviz);
      roscore_state = true;
      //set img on label
      ui.label_connect_state->setPixmap(QPixmap::fromImage(*img_robot_connect_state));
		}
  }
  else
  {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
           ui.line_edit_host->text().toStdString()) )
    {
			showNoMasterMessage();
      ui.treeWidget->setEnabled(false);
      roscore_state = false;
    }
    else
    {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
      ui.treeWidget->setEnabled(true);
      myrviz=new QRviz(ui.layout_rviz);
      roscore_state = true;
      //set img on label
      ui.label_connect_state->setPixmap(QPixmap::fromImage(*img_robot_connect_state));
		}
	}

  //激活相关按钮
  if(roscore_state) setButtonsEnable(true);
  else setButtonsEnable(false);

}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

void MainWindow::slot_treewidget_value_change(QString)
{
    myrviz->Set_FixedFrame(fixed_box->currentText());
}

void MainWindow::slot_display_grid(int state)
{
    bool enable=state>1?true:false;
    QStringList qli=Grid_Color_Box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_Grid(Cell_Count_Box->text().toInt(),color,enable);
}

void MainWindow::slot_display_tf(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_TF(enable);
}

void MainWindow::slot_display_laser(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_LaserScan(Laser_Topic_box->currentText(),enable);
}

void MainWindow::slot_display_Map(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_Map(Map_Topic_box->currentText(),Map_Color_Scheme_box->currentText(),enable);
}

void MainWindow::slot_display_marker(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_Marker(Marker_Topic_box->currentText(),enable);
}

void MainWindow::slot_ctrl_btn_press()
{
  QPushButton* btn=qobject_cast<QPushButton*> (sender());
  char k=btn->text().toStdString()[0];
  float linear=ui.label_linera->text().toFloat()*0.01;
  float angular=ui.label_raw->text().toFloat()*0.01;

  switch (k) {
    case 'i':
      qnode.set_cmd_vel('i',linear,angular);
      break;
    case 'u':
      qnode.set_cmd_vel('u',linear,angular);
      break;
    case 'o':
      qnode.set_cmd_vel('o',linear,angular);
      break;
    case 'j':
      qnode.set_cmd_vel('j',linear,angular);
      break;
    case 'l':
      qnode.set_cmd_vel('l',linear,angular);
      break;
    case 'm':
      qnode.set_cmd_vel('m',linear,angular);
      break;
    case ',':
      qnode.set_cmd_vel(',',linear,angular);
      break;
    case '.':
      qnode.set_cmd_vel('.',linear,angular);
      break;
    case 'k':
      qnode.set_cmd_vel('k',linear,angular);
      break;
  }
}

void MainWindow::slot_ctrl_btn_release()
{
  qnode.set_cmd_vel('k',0.0,0.0);
}

void MainWindow::slot_linera_value_change(int value)
{
    ui.label_linera->setText(QString::number(value));
}

void MainWindow::slot_raw_value_change(int value)
{
    ui.label_raw->setText(QString::number(value));
}

void MainWindow::slot_update_onepoint_nav_pos(double x,double y,double z,double w)
{
  //qDebug() << "slot_update_onepoint_nav_pos";

  ui.label_pose_x->setText(QString::number(x));
  ui.label_pose_y->setText(QString::number(y));
  ui.label_pose_z->setText(QString::number(z));
  ui.label_pose_w->setText(QString::number(w));

  unsigned long switch_num = 0;

  if(     ui.A01_rad->isChecked()) switch_num = 0;
  else if(ui.A02_rad->isChecked()) switch_num = 1;
  else if(ui.A03_rad->isChecked()) switch_num = 2;
  else if(ui.B01_rad->isChecked()) switch_num = 3;
  else if(ui.B02_rad->isChecked()) switch_num = 4;
  else if(ui.B03_rad->isChecked()) switch_num = 5;
  else if(ui.C01_rad->isChecked()) switch_num = 6;
  else if(ui.C02_rad->isChecked()) switch_num = 7;
  else if(ui.C03_rad->isChecked()) switch_num = 8;
  else if(ui.D01_rad->isChecked()) switch_num = 9;
  else if(ui.D02_rad->isChecked()) switch_num = 10;
  else if(ui.D03_rad->isChecked()) switch_num = 11;

  single_nav_goals[switch_num].x=x;
  single_nav_goals[switch_num].y=y;
  single_nav_goals[switch_num].z=z;
  single_nav_goals[switch_num].w=w;

}

void MainWindow::slot_update_back_nav_pos(double x,double y,double z,double w)
{
  //qDebug() << "slot_update_back_nav_pos";

  ui.label_pose_x->setText(QString::number(x));
  ui.label_pose_y->setText(QString::number(y));
  ui.label_pose_z->setText(QString::number(z));
  ui.label_pose_w->setText(QString::number(w));

  back_pose.x=x;
  back_pose.y=y;
  back_pose.z=z;
  back_pose.w=w;
  //通知-自动关闭退出
//  QMessageBox *m_box = new QMessageBox(QMessageBox::Information,QString("返航位置"),QString("返航点已刷新"));
//  QTimer::singleShot(700,m_box,SLOT(accept()));
//  m_box->exec();

}

void MainWindow::slot_update_points_nav_pos(double x,double y,double z,double w)
{
  //qDebug() << "slot_update_points_nav_pos";

  static bool fristFlag = false;
  ui.label_pose_x->setText(QString::number(x));
  ui.label_pose_y->setText(QString::number(y));
  ui.label_pose_z->setText(QString::number(z));
  ui.label_pose_w->setText(QString::number(w));

  MyPose tempPose;
  tempPose.x=x;
  tempPose.y=y;
  tempPose.z=z;
  tempPose.w=w;


  //清空历史数据
  if(fristFlag==0)
  {
    points_nav_goals.resize(0);
    fristFlag = true;
  }
  points_nav_goals.push_back(tempPose);

  //将导航路径画出来
  drawNavPath();
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

//读取历史设置
void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "robot_one");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
    //my setting
    ui.textEdit_new_map->setText(settings.value("textEdit_new_map",QString("default")).toString());
    ui.textEdit_save_map->setText(settings.value("textEdit_save_map",QString("default")).toString());
    ui.textEdit_launch_nav->setText(settings.value("textEdit_launch_nav",QString("default")).toString());
    ui.textEdit_stop_nav->setText(settings.value("textEdit_stop_nav",QString("default")).toString());

    //读-自定义单点导航按钮名称
    ui.textEdit_A01->setText(settings.value("textEdit_A01",QString("A01")).toString());
    ui.textEdit_A02->setText(settings.value("textEdit_A02",QString("A02")).toString());
    ui.textEdit_A03->setText(settings.value("textEdit_A03",QString("A03")).toString());
    ui.textEdit_B01->setText(settings.value("textEdit_B01",QString("B01")).toString());
    ui.textEdit_B02->setText(settings.value("textEdit_B02",QString("B02")).toString());
    ui.textEdit_B03->setText(settings.value("textEdit_B03",QString("B03")).toString());
    ui.textEdit_C01->setText(settings.value("textEdit_C01",QString("C01")).toString());
    ui.textEdit_C02->setText(settings.value("textEdit_C02",QString("C02")).toString());
    ui.textEdit_C03->setText(settings.value("textEdit_C03",QString("C03")).toString());
    ui.textEdit_D01->setText(settings.value("textEdit_D01",QString("D01")).toString());
    ui.textEdit_D02->setText(settings.value("textEdit_D02",QString("D02")).toString());
    ui.textEdit_D03->setText(settings.value("textEdit_D03",QString("D03")).toString());

    //读-单点导航位置
    for (unsigned long i=0;i<single_nav_goals.size();++i)
    {
       single_nav_goals[i].x = settings.value("onepoint_nav_"+QString::number(i)+"_x",0.0).toDouble();
       single_nav_goals[i].y = settings.value("onepoint_nav_"+QString::number(i)+"_y",0.0).toDouble();
       single_nav_goals[i].z = settings.value("onepoint_nav_"+QString::number(i)+"_z",0.0).toDouble();
       single_nav_goals[i].w = settings.value("onepoint_nav_"+QString::number(i)+"_w",0.0).toDouble();
    }
    //读-返航点位置
    back_pose.x = settings.value("back_pose_x",0.0).toDouble();
    back_pose.y = settings.value("back_pose_y",0.0).toDouble();
    back_pose.z = settings.value("back_pose_z",0.0).toDouble();
    back_pose.w = settings.value("back_pose_w",0.0).toDouble();

    //读-单点导航位置
    points_nav_goals.resize(settings.value("points_nav_size",0.0).toUInt());
//    qDebug() << settings.value("points_nav_size",0.0).toUInt();
    for (unsigned long i=0;i<points_nav_goals.size();++i)
    {
       points_nav_goals[i].x = settings.value("points_nav_"+QString::number(i)+"_x",0.0).toDouble();
       points_nav_goals[i].y = settings.value("points_nav_"+QString::number(i)+"_y",0.0).toDouble();
       points_nav_goals[i].z = settings.value("points_nav_"+QString::number(i)+"_z",0.0).toDouble();
       points_nav_goals[i].w = settings.value("points_nav_"+QString::number(i)+"_w",0.0).toDouble();
    }

}

//保存设置
void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "robot_one");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

    //my settings
    settings.setValue("textEdit_new_map",ui.textEdit_new_map->toPlainText());
    settings.setValue("textEdit_save_map",ui.textEdit_save_map->toPlainText());
    settings.setValue("textEdit_launch_nav",ui.textEdit_launch_nav->toPlainText());
    settings.setValue("textEdit_stop_nav",ui.textEdit_stop_nav->toPlainText());

    //写-自定义单点导航按钮名称
    settings.setValue("textEdit_A01",ui.textEdit_A01->toPlainText());
    settings.setValue("textEdit_A02",ui.textEdit_A02->toPlainText());
    settings.setValue("textEdit_A03",ui.textEdit_A03->toPlainText());
    settings.setValue("textEdit_B01",ui.textEdit_B01->toPlainText());
    settings.setValue("textEdit_B02",ui.textEdit_B02->toPlainText());
    settings.setValue("textEdit_B03",ui.textEdit_B03->toPlainText());
    settings.setValue("textEdit_C01",ui.textEdit_C01->toPlainText());
    settings.setValue("textEdit_C02",ui.textEdit_C02->toPlainText());
    settings.setValue("textEdit_C03",ui.textEdit_C03->toPlainText());
    settings.setValue("textEdit_D01",ui.textEdit_D01->toPlainText());
    settings.setValue("textEdit_D02",ui.textEdit_D02->toPlainText());
    settings.setValue("textEdit_D03",ui.textEdit_D03->toPlainText());

    //写-单点导航位置
    for (unsigned long i=0;i<single_nav_goals.size();++i)
    {
       settings.setValue("onepoint_nav_"+QString::number(i)+"_x",QVariant(single_nav_goals[i].x));
       settings.setValue("onepoint_nav_"+QString::number(i)+"_y",QVariant(single_nav_goals[i].y));
       settings.setValue("onepoint_nav_"+QString::number(i)+"_z",QVariant(single_nav_goals[i].z));
       settings.setValue("onepoint_nav_"+QString::number(i)+"_w",QVariant(single_nav_goals[i].w));
    }
    //写-返航点位置
    settings.setValue("back_pose_x",QVariant(back_pose.x));
    settings.setValue("back_pose_y",QVariant(back_pose.y));
    settings.setValue("back_pose_z",QVariant(back_pose.z));
    settings.setValue("back_pose_w",QVariant(back_pose.w));
    //写-多点导航位置
    settings.setValue("points_nav_size",QVariant(static_cast<double>(points_nav_goals.size())));
    for (unsigned long i=0;i<points_nav_goals.size();++i)
    {
       settings.setValue("points_nav_"+QString::number(i)+"_x",QVariant(points_nav_goals[i].x));
       settings.setValue("points_nav_"+QString::number(i)+"_y",QVariant(points_nav_goals[i].y));
       settings.setValue("points_nav_"+QString::number(i)+"_z",QVariant(points_nav_goals[i].z));
       settings.setValue("points_nav_"+QString::number(i)+"_w",QVariant(points_nav_goals[i].w));
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace robot_one

//输出信息
void robot_one::MainWindow::slot_quick_output()
{
      ui.textEdit_bash_output->append("<font color=\"#FF0000\">"+new_map_cmd->readAllStandardError()+"</font>");
      ui.textEdit_bash_output->append("<font color=\"#FFFFFF\">"+new_map_cmd->readAllStandardOutput()+"</font>");

      ui.textEdit_bash_output->append("<font color=\"#FF0000\">"+save_map_cmd->readAllStandardError()+"</font>");
      ui.textEdit_bash_output->append("<font color=\"#FFFFFF\">"+save_map_cmd->readAllStandardOutput()+"</font>");

      ui.textEdit_bash_output->append("<font color=\"#FF0000\">"+launch_onepoint_nav_cmd->readAllStandardError()+"</font>");
      ui.textEdit_bash_output->append("<font color=\"#FFFFFF\">"+launch_onepoint_nav_cmd->readAllStandardOutput()+"</font>");

      ui.textEdit_bash_output->append("<font color=\"#FF0000\">"+launch_stop_nav_cmd->readAllStandardError()+"</font>");
      ui.textEdit_bash_output->append("<font color=\"#FFFFFF\">"+launch_stop_nav_cmd->readAllStandardOutput()+"</font>");
}

//new map btn
void robot_one::MainWindow::on_new_map_btn_clicked()
{
  new_map_cmd->start("bash");
  new_map_cmd->waitForStarted();        //等待启动完成
  new_map_cmd->write(ui.textEdit_new_map->toPlainText().toLocal8Bit()+'\n');

  connect(new_map_cmd,SIGNAL(readyReadStandardError()),this, SLOT(slot_quick_output()));
  connect(new_map_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_quick_output()));

  //自动设置grid map check
  Grid_Check->setChecked(true);
  Map_Check->setChecked(true);

}

//save map btn
void robot_one::MainWindow::on_save_map_btn_clicked()
{
  save_map_cmd->start("bash");
  save_map_cmd->waitForStarted();                         //等待启动完成
  save_map_cmd->write(ui.textEdit_save_map->toPlainText().toLocal8Bit()+'\n');

  connect(save_map_cmd,SIGNAL(readyReadStandardError()),this, SLOT(slot_quick_output()));
  connect(save_map_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_quick_output()));
}

//edit map btn
void robot_one::MainWindow::on_edit_map_btn_clicked()
{
  MyPaint *w = new MyPaint();
  w->setWindowIcon(QIcon("://png/images/icons_index.png"));
  w->setWindowTitle("MAP 编辑器");
  //获取父窗口的位置，设置子窗口的位置
  QPoint globalPos = this->mapToGlobal(QPoint(0, 0));
  w->move(globalPos.x()+250, globalPos.y()+250);
  w->show();
}

//set nav init point
void robot_one::MainWindow::on_initpoint_nav_btn_clicked()
{
  myrviz->Set_Start_Pose();
}

//set nav one point
void robot_one::MainWindow::on_onepoint_nav_btn_clicked()
{
  bool switchFlag = false;

  if(ui.A01_rad->isChecked() || ui.A02_rad->isChecked() || ui.A03_rad->isChecked() ||
     ui.B01_rad->isChecked() || ui.B02_rad->isChecked() || ui.B03_rad->isChecked() ||
     ui.C01_rad->isChecked() || ui.C02_rad->isChecked() || ui.C03_rad->isChecked() ||
     ui.D01_rad->isChecked() || ui.D02_rad->isChecked() || ui.D03_rad->isChecked()){
    switchFlag = true;
  }

  if(!switchFlag)
  {
    QMessageBox msgBox;
    msgBox.setWindowTitle("单点选择警告");
    msgBox.setText("请先设置单点选择!    ");
    msgBox.exec();
  }
  else
  {
    myrviz->Set_single_nav_Pose();
  }
}

//set nav points
void robot_one::MainWindow::on_points_nav_btn_clicked()
{
  myrviz->Set_points_nav_Pose();
}

//set nav back point
void robot_one::MainWindow::on_backpoints_nav_btn_clicked()
{
  myrviz->Set_back_nav_Pose();
}

//launch one nav 启动 move_base + AMCL,如果默认启动了，无需再次启动
void robot_one::MainWindow::on_launch_nav_one_btn_clicked()
{
  launch_onepoint_nav_cmd->start("bash");
  launch_onepoint_nav_cmd->waitForStarted();                         //等待启动完成
  launch_onepoint_nav_cmd->write(ui.textEdit_launch_nav->toPlainText().toLocal8Bit()+'\n');

  connect(launch_onepoint_nav_cmd,SIGNAL(readyReadStandardError()),this, SLOT(slot_quick_output()));
  connect(launch_onepoint_nav_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_quick_output()));

  img_nav_state_1->load("://images/robot2.png");
  ui.label_nav_state_1->setPixmap(QPixmap::fromImage(*img_nav_state_1));
  //仅可以按一次
  ui.launch_nav_one_btn->setEnabled(false);
}

//stop nav 取消当前导航点
void robot_one::MainWindow::on_stop_nav_one_btn_clicked()
{
  launch_stop_nav_cmd->start("bash");
  launch_stop_nav_cmd->waitForStarted();                         //等待启动完成
  launch_stop_nav_cmd->write(ui.textEdit_stop_nav->toPlainText().toLocal8Bit()+'\n');

  connect(launch_stop_nav_cmd,SIGNAL(readyReadStandardError()),this, SLOT(slot_quick_output()));
  connect(launch_stop_nav_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_quick_output()));

  img_nav_state_1->load("://images/error.png");
  ui.label_nav_state_1->setPixmap(QPixmap::fromImage(*img_nav_state_1));
}

//back nav
void robot_one::MainWindow::on_back_nav_one_btn_clicked()
{
  //qDebug() << "on_back_nav_one_btn_clicked";
  qnode.set_goal(back_pose.x,back_pose.y,back_pose.z,back_pose.w);
  img_nav_state_1->load("://images/return.png");
  ui.label_nav_state_1->setPixmap(QPixmap::fromImage(*img_nav_state_1));
}

//launch points nav
void robot_one::MainWindow::on_launch_nav_points_btn_clicked()
{
  static bool once_flag = false;
  if(!once_flag)
  {
    //继续巡航
    ui.launch_nav_points_btn->setText("继续巡航");
    //创建线程开始标志
    qnode.pub_points_nav_start_flag(true);

    //qDebug()<<"points_nav_goals.size()"<<points_nav_goals.size();

    std::vector<geometry_msgs::Pose> tempPoseList;
    for (unsigned long i=0;i<points_nav_goals.size();++i) {
      geometry_msgs::Pose tempPose;
      tempPose.position.x    = points_nav_goals[i].x;
      tempPose.position.y    = points_nav_goals[i].y;
      tempPose.orientation.z = points_nav_goals[i].z;
      tempPose.orientation.w = points_nav_goals[i].w;
      tempPoseList.push_back(tempPose);
    }

    qnode.get_points_nav_info(tempPoseList,
                              ui.lineEdit_points_nav_num->text().toInt(),
                              ui.lineEdit_points_nav_stop_time->text().toInt(),
                              ui.checkBox_nav_next_flag->isChecked());
    //qDebug()<<ui.checkBox_nav_next_flag->isChecked();

    once_flag = true;
  }
  else
  {
    //继续巡航任务
    qnode.set_stop_points_nav(false);
  }

  img_nav_state_2->load("://images/robot2.png");
  ui.label_nav_state_2->setPixmap(QPixmap::fromImage(*img_nav_state_2));
}

//stop nav 暂停当前巡航任务
void robot_one::MainWindow::on_stop_nav_points_btn_clicked()
{
  //暂停当前巡航任务
  qnode.set_stop_points_nav(true);
  img_nav_state_2->load("://images/error.png");
  ui.label_nav_state_2->setPixmap(QPixmap::fromImage(*img_nav_state_2));
}

//next flag on nav
void robot_one::MainWindow::on_next_nav_points_btn_clicked()
{
  qnode.set_points_nav_next_btn_click();

  img_nav_state_2->load("://images/Navigate.png");
  ui.label_nav_state_2->setPixmap(QPixmap::fromImage(*img_nav_state_2));
}

//back nav
void robot_one::MainWindow::on_back_nav_points_btn_clicked()
{
  //qDebug() << "on_back_nav_points_btn_clicked";
  qnode.set_goal(back_pose.x,back_pose.y,back_pose.z,back_pose.w);
  img_nav_state_2->load("://images/return.png");
  ui.label_nav_state_2->setPixmap(QPixmap::fromImage(*img_nav_state_2));
}

//用官方的写法可以不用写connect函数,可以直接触发槽函数
void robot_one::MainWindow::on_A01_btn_clicked()
{
  //qDebug() << "on_A01_btn_clicked";
  qnode.set_goal(single_nav_goals[0].x,single_nav_goals[0].y,single_nav_goals[0].z,single_nav_goals[0].w);
}

void robot_one::MainWindow::on_A02_btn_clicked()
{
  qnode.set_goal(single_nav_goals[1].x,single_nav_goals[1].y,single_nav_goals[1].z,single_nav_goals[1].w);
}

void robot_one::MainWindow::on_A03_btn_clicked()
{
  qnode.set_goal(single_nav_goals[2].x,single_nav_goals[2].y,single_nav_goals[2].z,single_nav_goals[2].w);
}

void robot_one::MainWindow::on_B01_btn_clicked()
{
  qnode.set_goal(single_nav_goals[3].x,single_nav_goals[3].y,single_nav_goals[3].z,single_nav_goals[3].w);
}

void robot_one::MainWindow::on_B02_btn_clicked()
{
  qnode.set_goal(single_nav_goals[4].x,single_nav_goals[4].y,single_nav_goals[4].z,single_nav_goals[4].w);
}

void robot_one::MainWindow::on_B03_btn_clicked()
{
  qnode.set_goal(single_nav_goals[5].x,single_nav_goals[5].y,single_nav_goals[5].z,single_nav_goals[5].w);
}

void robot_one::MainWindow::on_C01_btn_clicked()
{
  qnode.set_goal(single_nav_goals[6].x,single_nav_goals[6].y,single_nav_goals[6].z,single_nav_goals[6].w);
}

void robot_one::MainWindow::on_C02_btn_clicked()
{
  qnode.set_goal(single_nav_goals[7].x,single_nav_goals[7].y,single_nav_goals[7].z,single_nav_goals[7].w);
}

void robot_one::MainWindow::on_C03_btn_clicked()
{
  qnode.set_goal(single_nav_goals[8].x,single_nav_goals[8].y,single_nav_goals[8].z,single_nav_goals[8].w);
}

void robot_one::MainWindow::on_D01_btn_clicked()
{
  qnode.set_goal(single_nav_goals[9].x,single_nav_goals[9].y,single_nav_goals[9].z,single_nav_goals[9].w);
}

void robot_one::MainWindow::on_D02_btn_clicked()
{
  qnode.set_goal(single_nav_goals[10].x,single_nav_goals[10].y,single_nav_goals[10].z,single_nav_goals[10].w);
}

void robot_one::MainWindow::on_D03_btn_clicked()
{
  qnode.set_goal(single_nav_goals[11].x,single_nav_goals[11].y,single_nav_goals[11].z,single_nav_goals[11].w);
}

void robot_one::MainWindow::on_checkBox_nav_next_flag_stateChanged(int arg1)
{
  ui.next_nav_points_btn->setEnabled(arg1);
}
