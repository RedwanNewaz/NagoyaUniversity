#include <ros/ros.h>
#include <QApplication>
#include "mainwindow.h"




int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosbag_decoder_node");

    ROS_INFO("Hello world!");
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
