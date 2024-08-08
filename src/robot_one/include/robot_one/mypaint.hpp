#ifndef MYPAINT_HPP
#define MYPAINT_HPP

#include <QMainWindow>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QVector>
#include <QPoint>
#include <QToolBar>
#include <QAction>
#include <QPalette>
#include <QColor>
#include <QMenu>
#include <QFileDialog>
#include <QTextEdit>
#include <QPen>

class MyPaint : public QMainWindow
{
    Q_OBJECT
public:
    explicit MyPaint(QWidget *parent = 0);
protected:
    void paintEvent(QPaintEvent *);            //重写窗体重绘事件
    void mousePressEvent(QMouseEvent *);       //重写鼠标按下事件
    void mouseReleaseEvent(QMouseEvent *);     //重写鼠标释放事件
    void mouseMoveEvent(QMouseEvent *);        //重写鼠标移动事件
    void contextMenuEvent(QContextMenuEvent *);//重写菜单事件
    void keyPressEvent(QKeyEvent *e);          //重写按键事件
public:
private:
    int _lpress;      //左键按下标志
    int _drag;        //拖拽标志
    int _drawType;    //描绘类型
    QMenu *_Rmenu;    //右键菜单
    int _openflag;    //打开图片
    QPixmap _pixmap;  //画布图片
    QTextEdit *_tEdit;//文本输入框
public:
    QVector<QVector<QPoint> > _lines; //线条集合(一条线条可包含多个线段)
    QVector<QRect> _rects;            //矩形集合
    QVector<QRect> _ellipse;          //椭圆集合
    QVector<QRect>  _line;            //直线集合
    QVector<QString>  _text;          //文字集合
    QVector<QPoint>  _tpoint;         //文字位置集合
    QVector<QVector<QPoint> > _eraser;//橡皮擦集合
    QVector<int>  _shape;             //图形类型集合，用于撤回功能
    QPoint _begin;                    //鼠标按下坐标、用于最后一个图形移动
signals:

public slots:
    void Lines();   //铅笔画线
    void SavePic(); //保存图片
    void Rects();   //画矩形
    void Ellipses();//画椭圆
    void Line();    //画直线
    void OpenPic(); //打开图片
    void Texts();   //文字
    void AddTexts();//添加文字
    void Eraser();  //橡皮擦
};

#endif // MYPAINT_HPP
