#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"Ground_Station");

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
