#include "roshandler.h"

RosHandler::RosHandler(QObject *parent) :
    QObject(parent)
{
    it = new image_transport::ImageTransport(nh);
    method_msg.data = "Surf";
    loadedObj_track_msg.data = false;
}

void RosHandler::main_func()
{
    image_transport::Subscriber object_tracking_sub = it->subscribe("/AUTMAV/surf_tracker_image", 1, &RosHandler::Object_Tracking_Image_Callback, this);
    image_transport::Subscriber color_tracking_sub = it->subscribe("/AUTMAV/color_tracker_image", 1, &RosHandler::Color_Tracking_Image_Callback, this);
    image_transport::Subscriber color_tracking_thresh_sub = it->subscribe("/AUTMAV/tresh_image", 1, &RosHandler::Color_Tracking_Thresh_Image_Callback, this);
    ros::Subscriber grid_sub = nh.subscribe<std_msgs::Float64MultiArray>("/AUTMAV/grid", 10, &RosHandler::grid_callback, this);
    ros::Subscriber commands_sub = nh.subscribe<Vangaurd::Guidance_Command>("guidance_pack", 10, &RosHandler::guidance_command_callback, this);
    ros::Subscriber nav_sub = nh.subscribe<Vangaurd::Navigation_Output>("/AUTMAV/Navigation", 10, &RosHandler::navigation_callback, this);
    ros::Subscriber string_sub = nh.subscribe<std_msgs::String>("/AUTMAV/string_msg", 10, &RosHandler::string_msg_callback, this);

    target_pub = nh.advertise<std_msgs::Float64MultiArray>("AUTMAV/target_point", 10);
    track_pub = nh.advertise<std_msgs::Bool>("/AUTMAV/track", 10);
    loadedObj_track_pub = nh.advertise<std_msgs::Bool>("/AUTMAV/loadedObj_track", 10);
    method_pub = nh.advertise<std_msgs::String>("/AUTMAV/detection_method", 10);
    hsv_pub = nh.advertise<std_msgs::UInt16MultiArray>("/AUTMAV/hsv_min_max", 10);
    upload_pub = nh.advertise<std_msgs::String>("/AUTMAV/upload",10);
    destination_name_pub = nh.advertise<std_msgs::String>("/AUTMAV/desti_dir",10);
    pid_pub = nh.advertise<std_msgs::Int16MultiArray>("/AUTMAV/pid_gains",10);
    bn_full_pub = nh.advertise<std_msgs::Int16>("/AUTMAV/block_number_full",10);
    bn_semi_pub = nh.advertise<std_msgs::Int16>("/AUTMAV/block_number_semi",10);
    command_full_pub = nh.advertise<std_msgs::String>("/AUTMAV/guidance_command_full",10);
    command_semi_pub = nh.advertise<std_msgs::String>("/AUTMAV/guidance_command_semi",10);
    obj_pub = it->advertise("AUTMAV/object", 10);

    ros::Timer timer = nh.createTimer(ros::Duration(0.0001),&RosHandler::timerCallback, this);

    ros::spin();
}

void RosHandler::onMethodSignal(String *m)
{
    method_msg.data = *m;
}

void RosHandler::onTrackSignal(bool t)
{
    track_msg.data = t;
}

void RosHandler::onHSVSignal(int hmin, int hmax, int smin, int smax, int vmin, int vmax)
{
    track_color = true;

    hsv_msg.data.resize(6);
    hsv_msg.data.at(0) = hmin;
    hsv_msg.data.at(1) = hmax;
    hsv_msg.data.at(2) = smin;
    hsv_msg.data.at(3) = smax;
    hsv_msg.data.at(4) = vmin;
    hsv_msg.data.at(5) = vmax;
}

void RosHandler::onStopColorTrackingSignal(bool s)
{
    track_color = false;
}

void RosHandler::onDirSignal(String *dir)
{
    Mat object = imread(dir->c_str());
    ros::Duration(0.05).sleep();
    sensor_msgs::ImagePtr obj_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", object).toImageMsg();
    obj_pub.publish(obj_msg);
    ros::Duration(0.05).sleep();
    loadedObj_track_msg.data = true;
    ros::spinOnce();
}

void RosHandler::onStopLoadedTrack(bool)
{
    loadedObj_track_msg.data = false;
}

void RosHandler::onUploadSignal(string *file_name)
{
    std_msgs::String msg;
    msg.data = *file_name;
    upload_pub.publish(msg);
}

void RosHandler::onDestinationDirSignl(string * desti_dir)
{
    std_msgs::String msg;
    msg.data = *desti_dir;
    destination_name_pub.publish(msg);
}

void RosHandler::onPIDGainsSignal(int PID_X_P, int PID_X_I, int PID_X_D,
                                  int PID_Y_P, int PID_Y_I, int PID_Y_D,
                                  int PID_Z_P, int PID_Z_I, int PID_Z_D,
                                  int PID_V_X_P, int PID_V_X_I, int PID_V_X_D,
                                  int PID_V_Y_P, int PID_V_Y_I, int PID_V_Y_D,
                                  int PID_V_Z_P, int PID_V_Z_I, int PID_V_Z_D,
                                  int PID_H_P, int PID_H_I, int PID_H_D,
                                  int PID_YAW_RATE_P, int PID_YAW_RATE_I, int PID_YAW_RATE_D)
{
    std_msgs::Int16MultiArray gains_msg;
    gains_msg.data.resize(24);
    gains_msg.data.at(0) = PID_X_P;
    gains_msg.data.at(1) = PID_X_I;
    gains_msg.data.at(2) = PID_X_D;
    gains_msg.data.at(3) = PID_Y_P;
    gains_msg.data.at(4) = PID_Y_I;
    gains_msg.data.at(5) = PID_Y_D;
    gains_msg.data.at(6) = PID_Z_P;
    gains_msg.data.at(7) = PID_Z_I;
    gains_msg.data.at(8) = PID_Z_D;
    gains_msg.data.at(9) = PID_V_X_P;
    gains_msg.data.at(10) = PID_V_X_I;
    gains_msg.data.at(11) = PID_V_X_D;
    gains_msg.data.at(12) = PID_V_Y_P;
    gains_msg.data.at(13) = PID_V_Y_I;
    gains_msg.data.at(14) = PID_V_Y_D;
    gains_msg.data.at(15) = PID_V_Z_P;
    gains_msg.data.at(16) = PID_V_Z_I;
    gains_msg.data.at(17) = PID_V_Z_D;
    gains_msg.data.at(18) = PID_H_P;
    gains_msg.data.at(19) = PID_H_I;
    gains_msg.data.at(20) = PID_H_D;
    gains_msg.data.at(21) = PID_YAW_RATE_P;
    gains_msg.data.at(22) = PID_YAW_RATE_I;
    gains_msg.data.at(23) = PID_YAW_RATE_D;

    pid_pub.publish(gains_msg);
}

void RosHandler::onBlockCommandFull(int block_number)
{
    std_msgs::Int16 bn_msg;
    bn_msg.data = block_number;
    bn_full_pub.publish(bn_msg);
}

void RosHandler::onBlockCommandSemi(int block_number)
{
    std_msgs::Int16 bn_msg;
    bn_msg.data = block_number;
    bn_semi_pub.publish(bn_msg);
}

void RosHandler::onCommandFullSignal(string *command)
{
    std_msgs::String command_msg;
    command_msg.data = command->data();
    command_full_pub.publish(command_msg);
}

void RosHandler::onCommandSemiSignal(string *command)
{
    std_msgs::String command_msg;
    command_msg.data = command->data();
    command_semi_pub.publish(command_msg);
}

void RosHandler::Object_Tracking_Image_Callback(const sensor_msgs::ImageConstPtr & msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
    Mat img = cv_ptr->image;

    emit ObjectTrackingImgSignal(&img);

    method_pub.publish(method_msg);
    track_pub.publish(track_msg);
    loadedObj_track_pub.publish(loadedObj_track_msg);

    ros::Duration(0.03).sleep();
}

void RosHandler::Color_Tracking_Image_Callback(const sensor_msgs::ImageConstPtr & msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
    Mat img = cv_ptr->image;
    emit ColorTrackingImgSignal(&img);

    if(track_color)
        hsv_pub.publish(hsv_msg);

    ros::Duration(0.03).sleep();
}

void RosHandler::Color_Tracking_Thresh_Image_Callback(const sensor_msgs::ImageConstPtr & msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    Mat img = cv_ptr->image;
    emit ColorTrackingThreshImgSignal(&img);

    ros::Duration(0.03).sleep();
}

void RosHandler::grid_callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
        vector <float> grid;
        for(int i=0 ; i < msg->data.size(); i++){
            grid.push_back(msg->data.at(i));
        }
        emit GridSignal(&grid);
}

void RosHandler::timerCallback(const ros::TimerEvent &e)
{
    qApp->processEvents();
}

void RosHandler::guidance_command_callback(const Vangaurd::Guidance_Command::ConstPtr &msg)
{
    emit CommandsSignal(msg->throttle, msg->roll, msg->pitch, msg->yaw, msg->arm);
}

void RosHandler::navigation_callback(const Vangaurd::Navigation_Output::ConstPtr &msg)
{
    emit NavigationSignal(msg->x, msg->y, msg->z,
                          msg->vx, msg->vy, msg->vz,
                          msg->phi, msg->theta, msg->psi,
                          msg->phi_dot, msg->theta_dot, msg->psi_dot);
}

void RosHandler::string_msg_callback(const std_msgs::String::ConstPtr &msg)
{
    string msg_content = msg->data;
    emit StringMsgSignal(&msg_content);
}
