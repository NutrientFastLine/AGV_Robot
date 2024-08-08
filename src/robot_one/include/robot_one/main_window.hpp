/**
 * @file /include/robot_one/main_window.hpp
 *
 * @brief Qt based gui for robot_one.
 *
 * @date November 2010
 **/
#ifndef robot_one_MAIN_WINDOW_H
#define robot_one_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "qrviz.hpp"
#include "mypaint.hpp"

#include <QTreeWidget>
#include <QTreeWidgetItem>

#include <QImage>
#include <QProcess>
#include <QComboBox>
#include <QSpinBox>
#include <QDebug>
#include <vector>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robot_one {

struct MyPose{
  double x;
  double y;
  double z;
  double w;
};

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

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
  void setButtonsEnable(bool);
  void drawNavPath();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
  void on_button_connect_clicked();
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void slot_treewidget_value_change(QString);
    void slot_display_grid(int);
    void slot_display_tf(int);
    void slot_display_laser(int);
    void slot_display_Map(int);
    void slot_display_marker(int);
    void slot_linera_value_change(int);
    void slot_raw_value_change(int);
    void slot_ctrl_btn_press();
    void slot_ctrl_btn_release();
    void slot_quick_output();
    void slot_update_onepoint_nav_pos(double,double,double,double);
    void slot_update_back_nav_pos(double,double,double,double);
    void slot_update_points_nav_pos(double,double,double,double);

  /******************************************
  ** Auto-connections (connectSlotsByName())用官方的写法可以不用写connect函数,可以直接触发槽函数,写了connect函数qt按钮的槽函数跳出两次
  *******************************************/
private slots:
    void on_new_map_btn_clicked();
    void on_save_map_btn_clicked();
    void on_edit_map_btn_clicked();
    void on_initpoint_nav_btn_clicked();
    void on_onepoint_nav_btn_clicked();
    void on_points_nav_btn_clicked();
    void on_backpoints_nav_btn_clicked();
    void on_launch_nav_one_btn_clicked();

    void on_stop_nav_one_btn_clicked();

    void on_back_nav_one_btn_clicked();

    void on_launch_nav_points_btn_clicked();

    void on_stop_nav_points_btn_clicked();

    void on_next_nav_points_btn_clicked();

    void on_back_nav_points_btn_clicked();

    void on_A01_btn_clicked();

    void on_A02_btn_clicked();

    void on_A03_btn_clicked();

    void on_B01_btn_clicked();

    void on_B02_btn_clicked();

    void on_B03_btn_clicked();

    void on_C01_btn_clicked();

    void on_C02_btn_clicked();

    void on_C03_btn_clicked();

    void on_D01_btn_clicked();

    void on_D02_btn_clicked();

    void on_D03_btn_clicked();

    void on_checkBox_nav_next_flag_stateChanged(int arg1);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  bool roscore_state;
  QRviz* myrviz;
  QComboBox* fixed_box;
  QSpinBox*  Cell_Count_Box;
  QComboBox* Grid_Color_Box;
  QComboBox* Laser_Topic_box;
  QComboBox* Map_Topic_box;
  QComboBox* Map_Color_Scheme_box;
  QComboBox* Marker_Topic_box;
  QCheckBox* Grid_Check;
  QCheckBox* Map_Check;

  QProcess *new_map_cmd;
  QProcess *save_map_cmd;
  QProcess *launch_onepoint_nav_cmd;
  QProcess *launch_stop_nav_cmd;
};

}  // namespace robot_one

#endif // robot_one_MAIN_WINDOW_H
