#ifndef ROSHANDLER_H
#define ROSHANDLER_H

#include <QObject>
#include <QtCore>
#include <QThread>
#include <QString>
#include <QDebug>
#include <QMutex>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "autmav_msgs/Guidance_Command.h"
#include "autmav_msgs/Navigation_Output.h"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <string.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

class RosHandler : public QObject
{
    Q_OBJECT
public:
    explicit RosHandler(QObject *parent = 0);

private:
    ros::NodeHandle nh;
    image_transport::ImageTransport *it;

    void Object_Tracking_Image_Callback(const sensor_msgs::ImageConstPtr&);
    void Color_Tracking_Image_Callback(const sensor_msgs::ImageConstPtr&);
    void Color_Tracking_Thresh_Image_Callback(const sensor_msgs::ImageConstPtr&);
    void grid_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& e);
    void guidance_command_callback(const Vangaurd::Guidance_Command::ConstPtr& msg);
    void navigation_callback(const Vangaurd::Navigation_Output::ConstPtr& msg);
    void string_msg_callback(const std_msgs::String::ConstPtr& msg);

    ros::Publisher target_pub;
    ros::Publisher track_pub;
    ros::Publisher loadedObj_track_pub;
    ros::Publisher hsv_pub;
    ros::Publisher method_pub;
    ros::Publisher upload_pub;
    ros::Publisher destination_name_pub;
    ros::Publisher pid_pub;
    ros::Publisher bn_full_pub;
    ros::Publisher bn_semi_pub;
    ros::Publisher command_full_pub;
    ros::Publisher command_semi_pub;
    image_transport::Publisher obj_pub;

    bool track_color = false;

    std_msgs::UInt16MultiArray hsv_msg;
    std_msgs::String method_msg;
    std_msgs::Bool track_msg;
    std_msgs::Bool loadedObj_track_msg;

signals:
    void ObjectTrackingImgSignal(Mat*);
    void ColorTrackingImgSignal(Mat*);
    void ColorTrackingThreshImgSignal(Mat*);
    void GridSignal(vector <float> *);
    void CommandsSignal(float, float, float, float, bool);
    void NavigationSignal(float, float, float,
                          float, float, float,
                          float, float, float,
                          float, float, float);
    void StringMsgSignal(string*);

public slots:
    void main_func();
    void onMethodSignal(String*);
    void onTrackSignal(bool);
    void onHSVSignal(int,int,int,int,int,int);
    void onStopColorTrackingSignal(bool);
    void onDirSignal(String*);
    void onStopLoadedTrack(bool);
    void onUploadSignal(string *);
    void onDestinationDirSignl(string *);
    void onPIDGainsSignal(int,int,int,
                        int,int,int,
                        int,int,int,
                        int,int,int,
                        int,int,int,
                        int,int,int,
                        int,int,int,
                        int,int,int);
    void onBlockCommandFull(int);
    void onBlockCommandSemi(int);
    void onCommandFullSignal(string *);
    void onCommandSemiSignal(string *);

};

#endif // ROSHANDLER_H
