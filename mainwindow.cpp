#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    layout = new QVBoxLayout;

    //this->setStyleSheet("QMainWindow { background-color: rgb(128,128,128); }");
    this->setWindowState(Qt::WindowMaximized);
    this->setWindowTitle("AUTMAV Ground Station");
    ui->GrabImgCheckbox->setChecked(true);
    ui->LoadObjectCheckbox->setChecked(false);
    ui->LoadObjectAndStart->setEnabled(false);
    ui->LoadStopTracking->setEnabled(false);
    ui->LoadObjectDir->setEnabled(false);
    ui->LoadObjectBrowse->setEnabled(false);
    ui->GrabImgCheckbox->setEnabled(true);
    ui->StopTracking->setEnabled(true);
    ui->SurfCheckbox->setChecked(true);
    /*ui->SurfCheckbox->setEnabled(false);
    ui->SiftCheckbox->setEnabled(false);
    ui->OrbCheckbox->setEnabled(false);
    ui->FASTOrbCheckBox->setEnabled(false);*/
    ui->h_minTrackbar->setRange(0,255);
    ui->s_minTrackbar->setRange(0,255);
    ui->v_minTrackbar->setRange(0,255);
    ui->h_maxTrackbar->setRange(0,255);
    ui->s_maxTrackbar->setRange(0,255);
    ui->v_maxTrackbar->setRange(0,255);
    ui->h_maxVal->setNum(255);
    ui->h_maxTrackbar->setValue(255);
    ui->s_maxVal->setNum(255);
    ui->s_maxTrackbar->setValue(255);
    ui->v_maxVal->setNum(255);
    ui->v_maxTrackbar->setValue(255);
    ui->RollValTO->setEnabled(false);
    ui->PitchValTO->setEnabled(false);
    ui->KnownPoseLX->setEnabled(false);
    ui->KnownPoseLY->setEnabled(false);
    ui->RollValL->setEnabled(false);
    ui->PitchValL->setEnabled(false);
    ui->HMarkerL->setEnabled(false);
    ui->HMarkerDir->setEnabled(false);
    ui->HMarkerLBrowse->setEnabled(false);
    ui->ObjectTrackingL->setEnabled(false);
    ui->ObjectTrackingLBrowse->setEnabled(false);
    ui->ObjectTrackingLDir->setEnabled(false);
    ui->ColorTrackingL->setEnabled(false);
    ui->ColorTrackingLDir->setEnabled(false);
    ui->ColorTrackingLBrowse->setEnabled(false);
    ui->RateValPH->setEnabled(false);
    ui->TurnsValPH->setEnabled(false);
    ui->LimitAnglePH->setEnabled(false);
    ui->CamAngL->setEnabled(false);
    ui->ObjTrackingCamAngle->setEnabled(false);
    ui->GenStatThrottle->setStyleSheet("background-color: black;");
    ui->GenStatThrottle->setPalette(Qt::red);
    ui->GenStatThrottle->display(0);
    ui->GenStatRoll->display(0);
    ui->GenStatRoll->setStyleSheet("background-color: black;");
    ui->GenStatRoll->setPalette(Qt::red);
    ui->GenStatPitch->display(0);
    ui->GenStatPitch->setStyleSheet("background-color: black;");
    ui->GenStatPitch->setPalette(Qt::red);
    ui->GenStatYaw->display(0);
    ui->GenStatYaw->setStyleSheet("background-color: black;");
    ui->GenStatYaw->setPalette(Qt::red);
    ui->PID_X_P_VAL->setValue(10);
    ui->PID_X_I_VAL->setValue(10);
    ui->PID_X_D_VAL->setValue(0);
    ui->PID_Y_P_VAL->setValue(10);
    ui->PID_Y_I_VAL->setValue(10);
    ui->PID_Y_D_VAL->setValue(0);
    ui->PID_Z_P_VAL->setValue(10);
    ui->PID_Z_I_VAL->setValue(10);
    ui->PID_Z_D_VAL->setValue(0);
    ui->PID_V_X_P_VAL->setValue(10);
    ui->PID_V_X_I_VAL->setValue(10);
    ui->PID_V_X_D_VAL->setValue(0);
    ui->PID_V_Y_P_VAL->setValue(10);
    ui->PID_V_Y_I_VAL->setValue(10);
    ui->PID_V_Y_D_VAL->setValue(0);
    ui->PID_V_Z_P_VAL->setValue(10);
    ui->PID_V_Z_I_VAL->setValue(10);
    ui->PID_V_Z_D_VAL->setValue(0);
    ui->PID_H_P_VAL->setValue(10);
    ui->PID_H_I_VAL->setValue(10);
    ui->PID_H_D_VAL->setValue(0);
    ui->PID_YAW_RATE_P_VAL->setValue(10);
    ui->PID_YAW_RATE_I_VAL->setValue(10);
    ui->PID_YAW_RATE_D_VAL->setValue(0);
    ui->BlocksManagementFull->setEnabled(false);
    ui->StartFull->setEnabled(false);
    ui->CancelCurrBlockFull->setEnabled(false);
    ui->ContinueFull->setEnabled(false);
    ui->StandByFull->setEnabled(false);
    ui->ShutDownFull->setEnabled(false);
    ui->BlocksManagementSemi->setEnabled(false);
    ui->CancelCurrBlockSemi->setEnabled(false);
    ui->ContinueSemi->setEnabled(false);
    ui->ShutDownSemi->setEnabled(false);
    ui->GenStatArmed->setStyleSheet("color: red;");

    rh = new RosHandler;
    connect(rh,SIGNAL(ObjectTrackingImgSignal(Mat*)),this,SLOT(onObjectTrackingImgSignal(Mat*)));
    connect(rh,SIGNAL(ColorTrackingImgSignal(Mat*)),this,SLOT(onColorTrackingImgSignal(Mat*)));
    connect(rh,SIGNAL(GridSignal(vector<float>*)),this,SLOT(onGridSignal(vector<float>*)));
    connect(rh,SIGNAL(ColorTrackingThreshImgSignal(Mat*)),this,SLOT(onColorTrackingThreshImgSignal(Mat*)));
    connect(rh,SIGNAL(CommandsSignal(float,float,float,float,bool)),this,SLOT(onCommandsSignal(float,float,float,float,bool)));
    connect(rh,SIGNAL(NavigationSignal(float,float,float,float,float,float,float,float,float,float,float,float)),
            this,SLOT(onNavigationSignal(float,float,float,float,float,float,float,float,float,float,float,float)));
    connect(rh,SIGNAL(StringMsgSignal(string*)),this,SLOT(onStringMsgSignal(string*)));
    connect(this,SIGNAL(TrackSignal(bool)),rh,SLOT(onTrackSignal(bool)));
    connect(this,SIGNAL(DirSignal(String*)),rh,SLOT(onDirSignal(String*)));
    connect(this,SIGNAL(StopLoadedTrack(bool)),rh,SLOT(onStopLoadedTrack(bool)));
    connect(this,SIGNAL(UploadSignal(string*)),rh,SLOT(onUploadSignal(string*)));
    connect(this,SIGNAL(DestinationNameSignal(string*)),rh,SLOT(onDestinationDirSignl(string*)));
    connect(this,SIGNAL(HSVSignal(int,int,int,int,int,int)),rh,SLOT(onHSVSignal(int,int,int,int,int,int)));
    connect(this,SIGNAL(StopColorTrackingSignal(bool)),rh,SLOT(onStopColorTrackingSignal(bool)));
    connect(this,SIGNAL(PIDGainsSignal(int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int)),
            rh,SLOT(onPIDGainsSignal(int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int,int)));
    connect(this,SIGNAL(BlockCommandFull(int)),rh,SLOT(onBlockCommandFull(int)));
    connect(this,SIGNAL(BlockCommandSemi(int)),rh,SLOT(onBlockCommandSemi(int)));
    connect(this,SIGNAL(CommandFullSignal(string*)),rh,SLOT(onCommandFullSignal(string*)));
    connect(this,SIGNAL(CommandSemiSignal(string*)),rh,SLOT(onCommandSemiSignal(string*)));

    mainThread = new QThread;
    rh->moveToThread(mainThread);
    connect(mainThread,SIGNAL(started()),rh,SLOT(main_func()));
    mainThread->start();

    /*WriteOT2YAML();
    WriteCT2YAML();
    WriteTO2YAML();
    WriteL2YAML();*/
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::WriteOT2YAML()
{
    string rm_dir = "rm " + obj_tracking_dir;
    system(rm_dir.c_str());
    FileStorage fs(obj_tracking_dir, FileStorage::WRITE);
    fs << "object_dir" << "object.png";
    fs << "method" << glob_method;
}

void MainWindow::WriteCT2YAML()
{
    string rm_dir = "rm " + color_tracking_dir;
    system(rm_dir.c_str());
    FileStorage fs(color_tracking_dir,FileStorage::WRITE);
    fs << "h_min" << ui->h_minTrackbar->value();
    fs << "h_max" << ui->h_maxTrackbar->value();
    fs << "s_min" << ui->s_minTrackbar->value();
    fs << "s_max" << ui->s_maxTrackbar->value();
    fs << "v_min" << ui->v_minTrackbar->value();
    fs << "v_max" << ui->v_maxTrackbar->value();
}

void MainWindow::WriteTO2YAML()
{
    string rm_dir = "rm " + take_off_dir;
    system(rm_dir.c_str());
    FileStorage fs(take_off_dir,FileStorage::WRITE);
    QString str1 = ui->TakeOffHeight->text();
    QString str2 = ui->TakeOffSpeed->text();
    fs << "height" << str1.toDouble();
    fs << "speed" << str2.toDouble();
    fs << "horizontal_mode" << to_horizontal_mode;
    QString str3 = ui->RollValTO->text();
    QString str4 = ui->PitchValTO->text();
    fs << "roll" << str3.toDouble();
    fs << "pitch" << str4.toDouble();
    fs << "heading_mode" << to_heading_mode;
}

void MainWindow::WriteL2YAML()
{
    string rm_dir = "rm " + landing_dir;
    system(rm_dir.c_str());
    FileStorage fs(landing_dir,FileStorage::WRITE);

    FileStorage fs1;

    string object_dir_str = "-", method_str = "-";
    int h_min = 0, h_max = 255, s_min = 0, s_max = 255, v_min = 0, v_max = 255;

    if(ui->HMarkerL->isChecked()){
        fs1 = FileStorage(ui->HMarkerDir->text().toUtf8().constData(),FileStorage::READ);

    };
    if(ui->ObjectTrackingL->isChecked()){
        fs1 = FileStorage(ui->ObjectTrackingLDir->text().toUtf8().constData(),FileStorage::READ);
        fs1["object_dir"] >> object_dir_str;
        fs1["method"] >> method_str;
    }
    if(ui->ColorTrackingL->isChecked()){
        fs1 = FileStorage(ui->ColorTrackingLDir->text().toUtf8().constData(),FileStorage::READ);
        fs1["h_min"] >> h_min;
        fs1["h_max"] >> h_max;
        fs1["s_min"] >> s_min;
        fs1["s_max"] >> s_max;
        fs1["v_min"] >> v_min;
        fs1["v_max"] >> v_max;
    }

    QString str1 = ui->LandingSpeed->text();
    QString str2 = ui->RollValL->text();
    QString str3 = ui->PitchValL->text();
    QString str4 = ui->KnownPoseLX->text();
    QString str5 = ui->KnownPoseLY->text();
    QString str6 = ui->ObjTrackingCamAngle->text();
    string prec_method = "";

    if(ui->HMarkerL->isChecked())
        prec_method = "H_Marker";
    else if(ui->ObjectTrackingL->isChecked())
        prec_method = "Object_Tracking";
    else if(ui->ColorTrackingL->isChecked())
        prec_method = "Color_Tracking";

    fs << "speed" << str1.toDouble();
    fs << "horizontal_mode" << l_horizontal_mode;
    fs << "heading_mode" << l_heading_mode;
    fs << "object_dir" << object_dir_str;
    fs << "method" << method_str;
    fs << "precision_method" << prec_method;
    fs << "roll" << str2.toDouble();
    fs << "pitch" << str3.toDouble();
    fs << "known_x" << str4.toDouble();
    fs << "known_y" << str5.toDouble();
    fs << "camera_angle" << str6.toDouble();
    fs << "h_min" << h_min;
    fs << "h_max" << h_max;
    fs << "s_min" << s_min;
    fs << "s_max" << s_max;
    fs << "v_min" << v_min;
    fs << "v_max" << v_max;
}

void MainWindow::WritePH2YAML()
{
    string rm_dir = "rm " + position_hold_dir;
    system(rm_dir.c_str());
    FileStorage fs(position_hold_dir,FileStorage::WRITE);

    fs << "horizontal_mode" << ph_horizontal_mode;
    fs << "heading_mode" << ph_heading_mode;
    fs << "stop_condition" << ph_stop_condition;
    fs << "global_angle" << ui->HeadingModeAnglePH->text().toDouble();
    fs << "rate" << ui->HeadingModeRatePH->text().toDouble();
    fs << "turns" << ui->TurnsValPH->text().toDouble();
    fs << "time" << ui->TimeValPH->text().toDouble();
    fs << "delta_angle" << ui->LimitAnglePH->text().toDouble();
}

void MainWindow::WriteHD2YAML()
{
    string rm_dir = "rm " + h_detection_dir;
    system(rm_dir.c_str());
    FileStorage fs(h_detection_dir,FileStorage::WRITE);

    fs << "marker_dir" << destination_name;
    fs << "method" << "Surf";
}

void MainWindow::addCommand()
{
    string command;
            if(blocks.back().first == "to"){
                    FileStorage fs(take_off_settings.at(blocks.back().second.first).first,FileStorage::READ);
                    float height = 0.0,speed = 0.0, roll = 0, pitch = 0;
                    string horizontal_mode, heading_mode;
                    fs["height"] >> height;
                    fs["speed"] >> speed;
                    fs["horizontal_mode"] >> horizontal_mode;
                    fs["heading_mode"] >> heading_mode;
                    fs["roll"] >> roll;
                    fs["pitch"] >> pitch;

                    stringstream convert;
                    convert << height;
                    string s_height = convert.str();
                    convert.str("");

                    convert << speed;
                    string s_speed = convert.str();
                    convert.str("");

                    convert << roll;
                    string s_roll = convert.str();
                    convert.str("");

                    convert << pitch;
                    string s_pitch = convert.str();
                    convert.str("");

                    convert << block_number;
                    string s_block_number = convert.str();
                    convert.str("");

                    command = "\ntake_off_" + s_block_number + ":\n   - { height:" + s_height + ", speed:" + s_speed + ", horizontal_mode:"
                            + horizontal_mode + ", heading_mode:" + heading_mode + ", roll:" + s_roll + ", pitch:" + s_pitch + " }\n";

                    block_number++;
                }
           else if(blocks.back().first == "l"){
                    FileStorage fs(landing_settings.at(blocks.back().second.first).first,FileStorage::READ);
                    float speed = 0.0, roll = 0.0, pitch = 0.0, x = 0.0, y = 0.0;
                    int h_min = 0, h_max = 255, s_min = 0, s_max = 255, v_min = 0, v_max = 255, angle = 0;
                    string object_dir_str, method_str, prec_method;
                    string hori_mode, head_mode;

                    fs["speed"] >> speed;
                    fs["horizontal_mode"] >> hori_mode;
                    fs["heading_mode"] >> head_mode;
                    fs["object_dir"] >> object_dir_str;
                    fs["method"] >> method_str;
                    fs["precision_method"] >> prec_method;
                    fs["roll"] >> roll;
                    fs["pitch"] >> pitch;
                    fs["known_x"] >> x;
                    fs["known_y"] >> y;
                    fs["camera_angle"] >> angle;
                    fs["h_min"] >> h_min;
                    fs["h_max"] >> h_max;
                    fs["s_min"] >> s_min;
                    fs["s_max"] >> s_max;
                    fs["v_min"] >> v_min;
                    fs["v_max"] >> v_max;

                    stringstream convert;
                    convert << speed;
                    string s_speed = convert.str();
                    convert.str("");

                    convert << h_min;
                    string s_h_min = convert.str();
                    convert.str("");

                    convert << h_max;
                    string s_h_max = convert.str();
                    convert.str("");

                    convert << s_min;
                    string s_s_min = convert.str();
                    convert.str("");

                    convert << s_max;
                    string s_s_max = convert.str();
                    convert.str("");

                    convert << v_min;
                    string s_v_min = convert.str();
                    convert.str("");

                    convert << v_max;
                    string s_v_max = convert.str();
                    convert.str("");

                    convert << roll;
                    string s_roll = convert.str();
                    convert.str("");

                    convert << pitch;
                    string s_pitch = convert.str();
                    convert.str("");

                    convert << x;
                    string s_x = convert.str();
                    convert.str("");

                    convert << y;
                    string s_y = convert.str();
                    convert.str("");

                    convert << angle;
                    string s_angle = convert.str();
                    convert.str("");

                    convert << block_number;
                    string s_block_number = convert.str();
                    convert.str("");

                    command = "\nlanding_" + s_block_number + ":\n   - { speed:" + s_speed + ", horizontal_mode:" + hori_mode + ", heading_mode:"
                            +  head_mode + ", object_dir:" + object_dir_str + ", method:" + method_str + ", precision_method:" + prec_method +
                            ", roll:" + s_roll + ", pitch:" + s_pitch + ", known_x:" + s_x + ", known_y:" + s_y + ", h_min:" + s_h_min + ", h_max:"
                            + s_h_max + ", s_min:" + s_s_min + ", s_max:" + s_s_max + ", v_min:" + s_v_min + ", v_max:"+ s_v_max + ", camera_angle:"
                            + s_angle + " }\n";

                    block_number++;
                }
            else if(blocks.back().first == "ph"){
                    FileStorage fs(position_hold_settings.at(blocks.back().second.first).first,FileStorage::READ);
                    string horizontal,heading,condition;
                    float global_angle = 0.0, rate = 0.0, turns = 0.0, time = 0.0, angle = 0.0;

                    fs["horizontal_mode"] >> horizontal;
                    fs["heading_mode"] >> heading;
                    fs["stop_condition"] >> condition;
                    fs["global_angle"] >> global_angle;
                    fs["rate"] >> rate;
                    fs["turns"] >> turns;
                    fs["time"] >> time;
                    fs["delta_angle"] >> angle;

                    stringstream convert;
                    convert << global_angle;
                    string s_global_angle = convert.str();
                    convert.str("");

                    convert << rate;
                    string s_rate = convert.str();
                    convert.str("");

                    convert << turns;
                    string s_turns = convert.str();
                    convert.str("");

                    convert << time;
                    string s_time = convert.str();
                    convert.str("");

                    convert << angle;
                    string s_angle = convert.str();
                    convert.str("");

                    convert << block_number;
                    string s_block_number = convert.str();
                    convert.str("");

                    command = "\nposition_hold_" + s_block_number + ":\n   - { heading:" + heading + ", horizontal_mode:"
                            + horizontal + ", stop_condition:" + condition + ", global_angle:" + s_global_angle + ", rate:"
                            + s_rate + ", turns:" + s_turns + ", time:" + s_time + ", delta_angle:" + s_angle + " }\n";

                    block_number++;
                }

    QString addition(command.c_str());
    ui->AutonomousPlain->moveCursor(QTextCursor::End);
    ui->AutonomousPlain->insertPlainText(addition);
}

void MainWindow::sendCommandFull(QToolButton *tb)
{
    for(int i=0; i<blocks.size(); i++){
        if(tb == blocks.at(i).second.second[1]){
            emit BlockCommandFull(i);
        }
    }
}

void MainWindow::sendCommandSemi(QToolButton *tb)
{
    for(int i=0; i<blocks.size(); i++){
        if(tb == blocks.at(i).second.second[2]){
            emit BlockCommandSemi(i);
        }
    }
}

void MainWindow::send_flight_plan()
{
    int is_sent = system("scp /home/hojat/GS_WS/src/Ground_Station_1-build/devel/lib/Ground_Station_1/flight_plan.yaml autmav@192.168.197.32:/home/autmav/ROS_CATKIN_WS/src/Vanguard");
    cout << is_sent << endl;
}

void MainWindow::onObjectTrackingImgSignal(Mat * img)
{
    int imgw;
    int imgh;

    if(tracking_tab == "obj"){
        imgw = ui->ObjTrackingImage->width();
        imgh = ui->ObjTrackingImage->height();
    }
    else{
        imgw = ui->HDetectionImage->width();
        imgh = ui->HDetectionImage->height();
    }

    cv::resize(*img,*img,{imgw,imgh});

    QPixmap p = QPixmap::fromImage(QImage((uchar*)img->data,img->cols,img->rows,img->step,QImage::Format_RGB888));

    QGraphicsScene *scene;
    scene = new QGraphicsScene(this);
    scene->addPixmap(p);

    if(tracking_tab == "obj")
        ui->ObjTrackingImage->setScene(scene);
    else if(tracking_tab == "h")
        ui->HDetectionImage->setScene(scene);
}

void MainWindow::onColorTrackingImgSignal(Mat * img)
{
    int imgw = ui->ColorTackingImage->size().width();
    int imgh = ui->ColorTackingImage->size().height();

    cv::resize(*img,*img,{imgw,imgh});

    QPixmap p = QPixmap::fromImage(QImage((uchar*)img->data,img->cols,img->rows,img->step,QImage::Format_RGB888));

    QGraphicsScene *scene;
    scene = new QGraphicsScene(this);
    scene->addPixmap(p);

    ui->ColorTackingImage->setScene(scene);
}

void MainWindow::onColorTrackingThreshImgSignal(Mat * img)
{
    int imgw = ui->ColorTrackingTreshImage->size().width();
    int imgh = ui->ColorTrackingTreshImage->size().height();

    cv::resize(*img,*img,{imgw,imgh});

    QPixmap p = QPixmap::fromImage(QImage((uchar*)img->data,img->cols,img->rows,img->step,QImage::Format_Indexed8));

    QGraphicsScene *scene;
    scene = new QGraphicsScene(this);
    scene->addPixmap(p);

    ui->ColorTrackingTreshImage->setScene(scene);
}

void MainWindow::onGridSignal(vector<float> * grid)
{
    int hgth = grid->at(0);
    int wth = grid->at(1);
    int curr_x = grid->at(3);
    int curr_y = grid->at(4);

    QPixmap *p;
    p = new QPixmap(wth,hgth);

    QPainter painter(p);

    QColor *color = new QColor(0,0,0);

    for(int i=0; i<hgth; i++){
        for(int j=0; j<wth; j++){

            if(grid->at(i*wth+j+5) == 2000)
                color = new QColor(0,0,0);
            else if(grid->at(i*wth+j+5) == 2001)
                color = new QColor(255,0,0);
            else
                color = new QColor(192,192,192);

            QPen pen(*color);
            painter.setPen(pen);

            painter.drawPoint(i,j);

        }
    }

    color = new QColor(0,255,0);
    QPen pen(*color);
    painter.setPen(pen);

    painter.drawEllipse({curr_x,curr_y},5,5);

    QGraphicsScene *scene;
    scene = new QGraphicsScene(this);
    scene->addPixmap(*p);

    ui->MonitorSemi->setScene(scene);
}

void MainWindow::onCommandsSignal(float throttle, float roll, float pitch, float yaw, bool arm)
{
    ui->GenStatThrottle->display((int)throttle);
    ui->GenStatRoll->display((int)roll);
    ui->GenStatPitch->display((int)pitch);
    ui->GenStatYaw->display((int)yaw);
    if(arm){
        ui->GenStatArmed->setText("armed!");
        ui->GenStatArmed->setStyleSheet("color: blue;");
    }
    else{
        ui->GenStatArmed->setText("not armed!");
        ui->GenStatArmed->setStyleSheet("color: red;");
    }
}

void MainWindow::onNavigationSignal(float x, float y, float z,
                                    float vx, float vy, float vz,
                                    float phi, float theta, float psi,
                                    float phi_dot, float theta_dot, float psi_dot)
{
   ui->GenStatX->setNum(x);
   ui->GenStatY->setNum(y);
   ui->GenStatZ->setNum(z);
   ui->GenStatVX->setNum(vx);
   ui->GenStatVY->setNum(vy);
   ui->GenStatVZ->setNum(vz);
   ui->GenStatPhi->setNum(phi);
   ui->GenStatTheta->setNum(theta);
   ui->GenStatPsi->setNum(psi);
   ui->GenStatPhiDot->setNum(phi_dot);
   ui->GenStatThetaDot->setNum(theta_dot);
   ui->GenStatPsiDot->setNum(psi_dot);
}

void MainWindow::onStringMsgSignal(string *terminal_str)
{
    ui->TerminalOutput->append(terminal_str->c_str());
}

void MainWindow::on_GrabObjAndStart_clicked()
{
    emit TrackSignal(true);

    ui->SurfCheckbox->setEnabled(true);
    ui->SiftCheckbox->setEnabled(true);
    ui->OrbCheckbox->setEnabled(true);
    ui->FASTOrbCheckBox->setEnabled(true);
}

void MainWindow::on_StopTracking_clicked()
{
    emit TrackSignal(false);

    ui->SurfCheckbox->setEnabled(false);
    ui->SiftCheckbox->setEnabled(false);
    ui->OrbCheckbox->setEnabled(false);
    ui->FASTOrbCheckBox->setEnabled(false);
}

void MainWindow::on_LoadObjectCheckbox_stateChanged(int arg1)
{
    bool state = ui->LoadObjectCheckbox->checkState();
    if(!state){
        ui->LoadObjectAndStart->setEnabled(false);
        ui->LoadStopTracking->setEnabled(false);
        ui->LoadObjectDir->setEnabled(false);
        ui->LoadObjectBrowse->setEnabled(false);
    }
    else{
        ui->LoadObjectAndStart->setEnabled(true);
        ui->LoadStopTracking->setEnabled(true);
        ui->LoadObjectDir->setEnabled(true);
        ui->LoadObjectBrowse->setEnabled(true);

        ui->GrabImgCheckbox->setChecked(false);

        ui->GrabObjAndStart->setEnabled(false);
        ui->StopTracking->setEnabled(false);
    }
}

void MainWindow::on_GrabImgCheckbox_stateChanged(int arg1)
{
    bool state = ui->GrabImgCheckbox->checkState();
    if(!state){
        ui->GrabObjAndStart->setEnabled(false);
        ui->StopTracking->setEnabled(false);
    }
    else{
        ui->GrabObjAndStart->setEnabled(true);
        ui->StopTracking->setEnabled(true);

        ui->LoadObjectCheckbox->setChecked(false);

        ui->LoadObjectAndStart->setEnabled(false);
        ui->LoadStopTracking->setEnabled(false);
        ui->LoadObjectDir->setEnabled(false);
        ui->LoadObjectBrowse->setEnabled(false);
    }
}

void MainWindow::on_LoadObjectBrowse_clicked()
{
    QString dir;
    dir = QFileDialog::getOpenFileName(this,"Opening object image");
    ui->LoadObjectDir->setText(dir);
}

void MainWindow::on_LoadObjectAndStart_clicked()
{
    if(ui->DestinationName->text().toUtf8().constData() != "")
        destination_name = ui->DestinationName->text().toUtf8().constData() ;
    emit DestinationNameSignal(&destination_name);

    String dir = ui->LoadObjectDir->text().toUtf8().constData();
    emit DirSignal(&dir);

    /*ui->SurfCheckbox->setEnabled(true);
    ui->SiftCheckbox->setEnabled(true);
    ui->OrbCheckbox->setEnabled(true);
    ui->FASTOrbCheckBox->setEnabled(true);*/
}

void MainWindow::on_LoadStopTracking_clicked()
{
    emit StopLoadedTrack(true);

    /*ui->SurfCheckbox->setEnabled(false);
    ui->SiftCheckbox->setEnabled(false);
    ui->OrbCheckbox->setEnabled(false);
    ui->FASTOrbCheckBox->setEnabled(false);*/
}

void MainWindow::on_SaveAsObjTracking_clicked()
{
    QString str = QFileDialog::getSaveFileName(this,"Saving settings");
    obj_tracking_dir = str.toUtf8().constData();
    WriteOT2YAML();
}

void MainWindow::on_SaveObjTracking_clicked()
{
    WriteOT2YAML();
}

void MainWindow::on_SurfCheckbox_clicked()
{
    bool state = ui->SurfCheckbox->checkState();
    if(!state){
        ui->SurfCheckbox->setChecked(true);
        String method = "Surf";
        emit MethodSignal(&method);
    }

    ui->OrbCheckbox->setChecked(false);
    ui->SiftCheckbox->setChecked(false);
    ui->FASTOrbCheckBox->setChecked(false);

    glob_method = "Surf";
}

void MainWindow::on_SiftCheckbox_clicked()
{
    bool state = ui->SiftCheckbox->checkState();
    if(!state){
        ui->SiftCheckbox->setChecked(true);
        String method = "Sift";
        emit MethodSignal(&method);
    }

    ui->SurfCheckbox->setChecked(false);
    ui->OrbCheckbox->setChecked(false);
    ui->FASTOrbCheckBox->setChecked(false);

    glob_method = "Sift";
}

void MainWindow::on_OrbCheckbox_clicked()
{
    bool state = ui->OrbCheckbox->checkState();
    if(!state){
        ui->OrbCheckbox->setChecked(true);
        String method = "Orb";
        emit MethodSignal(&method);
    }

    ui->SurfCheckbox->setChecked(false);
    ui->SiftCheckbox->setChecked(false);
    ui->FASTOrbCheckBox->setChecked(false);

    glob_method = "Orb";
}

void MainWindow::on_FASTOrbCheckBox_clicked()
{
    bool state = ui->FASTOrbCheckBox->checkState();
    if(!state){
        ui->FASTOrbCheckBox->setChecked(true);
        String method = "FASTOrb";
        emit MethodSignal(&method);
    }

    ui->SurfCheckbox->setChecked(false);
    ui->SiftCheckbox->setChecked(false);
    ui->OrbCheckbox->setChecked(false);

    glob_method = "FASTOrb";
}

void MainWindow::on_SaveAsColorTracking_clicked()
{
    QString str = QFileDialog::getSaveFileName(this,"Saving Color_Tracking settings");
    color_tracking_dir = str.toUtf8().constData();
    WriteCT2YAML();
}

void MainWindow::on_SaveColorTracking_clicked()
{
    WriteCT2YAML();
}

void MainWindow::on_SaveAsTakeOff_clicked()
{
    QString str = QFileDialog::getSaveFileName(this,"Saving Take_Off settings");
    take_off_dir = str.toUtf8().constData();
    WriteTO2YAML();
}

void MainWindow::on_SaveTakeOff_clicked()
{
    WriteTO2YAML();
}

void MainWindow::on_SaveAsLanding_clicked()
{
    QString str = QFileDialog::getSaveFileName(this,"Saving Landing settings");
    landing_dir = str.toUtf8().constData();
    WriteL2YAML();
}

void MainWindow::on_SaveLanding_clicked()
{
    WriteL2YAML();
}

void MainWindow::on_BrowseTeakeOff_clicked()
{
    QString str = QFileDialog::getOpenFileName(this,"Loading Take_Off settings");
    ui->TakeOffDir->setText(str);
    if(last_tos)
        take_off_settings.push_back({ str.toUtf8().constData(), false});
    else
       take_off_settings.back() = { str.toUtf8().constData(), false};
    last_tos = false;
}

void MainWindow::on_BrowseLanding_clicked()
{
    QString str = QFileDialog::getOpenFileName(this,"Loading Landing settings");
    ui->LandingDir->setText(str);
    if(last_ls)
        landing_settings.push_back({ str.toUtf8().constData(), false});
    else
        landing_settings.back() = { str.toUtf8().constData(), false};
    last_ls = false;
}

void MainWindow::on_BrowseBeams_clicked()
{

}

void MainWindow::on_PositionHoldBrowse_clicked()
{
    QString str = QFileDialog::getOpenFileName(this,"Loading Position Hold settings");
    ui->PositionHoldDir->setText(str);
    if(last_phs)
        position_hold_settings.push_back({str.toUtf8().constData(),false});
    else
        position_hold_settings.back() = { str.toUtf8().constData(), false};
    last_phs = false;
}

void MainWindow::on_AddTakeOff_clicked()
{
    take_off_settings.back().second = true;
    last_tos = true;

    stringstream convert;
    convert << take_off_settings.size() - 1;
    string text = "Take Off " + convert.str();
    QString str(text.c_str());

    QToolButton *button0 = new QToolButton;
    button0->setText(str);
    button0->setFixedSize(ui->BlocksManagementFull->width() - 30, 80);
    button0->setStyleSheet("color: blue;");
    ui->BlocksLayoutFull->addWidget(button0);

    QToolButton *button1 = new QToolButton;
    button1->setText(str);
    button1->setFixedSize(ui->BlocksManagementSemi->width() - 30, 80);
    button1->setStyleSheet("color: blue;");
    ui->BlocksLayoutSemi->addWidget(button1);

    vector <QToolButton*> temp(2);
    temp.at(0) = button0;
    temp.at(1) = button1;

    blocks.push_back({"to",{take_off_settings.size()-1, temp}});

    connect(button0,SIGNAL(clicked()),this,SLOT(on_ToolButtons_clicked_full()));
    connect(button1,SIGNAL(clicked()),this,SLOT(on_ToolButtons_clicked_semi()));

    ui->TakeOffDir->clear();

    addCommand();
}

void MainWindow::on_AddLanding_clicked()
{
    landing_settings.back().second = true;
    last_ls = true;

    stringstream convert;
    convert << landing_settings.size() - 1;
    string text = "Landing " + convert.str();
    QString str(text.c_str());

    QToolButton *button0 = new QToolButton;
    button0->setText(str);
    button0->setFixedSize(ui->BlocksManagementFull->width() - 30, 80);
    button0->setStyleSheet("color: blue;");
    ui->BlocksLayoutFull->addWidget(button0);

    QToolButton *button1 = new QToolButton;
    button1->setText(str);
    button1->setFixedSize(ui->BlocksManagementSemi->width() - 30, 80);
    button1->setStyleSheet("color: blue;");
    ui->BlocksLayoutSemi->addWidget(button1);

    vector <QToolButton*> temp(2);
    temp.at(0) = button0;
    temp.at(1) = button1;

    blocks.push_back({"l",{landing_settings.size()-1, temp}});

    connect(button0,SIGNAL(clicked()),this,SLOT(on_ToolButtons_clicked_full()));
    connect(button1,SIGNAL(clicked()),this,SLOT(on_ToolButtons_clicked_semi()));

    ui->LandingDir->clear();

    addCommand();
}

void MainWindow::on_AddBeams_clicked()
{

}

void MainWindow::on_AddPositionHold_clicked()
{
    position_hold_settings.back().second = true;
    last_phs = true;

    stringstream convert;
    convert << position_hold_settings.size() - 1;
    string text = "Position Hold " + convert.str();
    QString str(text.c_str());

    QToolButton *button0 = new QToolButton;
    button0->setText(str);
    button0->setFixedSize(ui->BlocksManagementFull->width() - 30, 80);
    button0->setStyleSheet("color: blue;");
    ui->BlocksLayoutFull->addWidget(button0);

    QToolButton *button1 = new QToolButton;
    button1->setText(str);
    button1->setFixedSize(ui->BlocksManagementSemi->width() - 30, 80);
    button1->setStyleSheet("color: blue;");
    ui->BlocksLayoutSemi->addWidget(button1);

    vector <QToolButton*> temp(2);

    temp.at(0) = button0;
    temp.at(1) = button1;

    blocks.push_back({"ph",{position_hold_settings.size()-1, temp}});

    connect(button0,SIGNAL(clicked()),this,SLOT(on_ToolButtons_clicked_full()));
    connect(button1,SIGNAL(clicked()),this,SLOT(on_ToolButtons_clicked_semi()));

    ui->PositionHoldDir->clear();

    addCommand();
}

void MainWindow::on_ToolButtons_clicked_full()
{
    QObject *obj = this->sender();
    QToolButton *clicked;
    clicked = qobject_cast<QToolButton*>(obj);
    sendCommandFull(clicked);
}

void MainWindow::on_ToolButtons_clicked_semi()
{
    QObject *obj = this->sender();
    QToolButton *clicked;
    clicked = qobject_cast<QToolButton*>(obj);
    sendCommandSemi(clicked);
}

void MainWindow::on_VelocityModeTO_clicked()
{
      bool checked = ui->VelocityModeTO->isChecked();
      if(checked){
          ui->RollValTO->setEnabled(false);
          ui->PitchValTO->setEnabled(false);
          to_horizontal_mode = "Zero_Velo";
      }
}

void MainWindow::on_AttidtudeModeTO_clicked()
{
    bool checked = ui->AttidtudeModeTO->isChecked();
    if(checked){
        ui->RollValTO->setEnabled(true);
        ui->PitchValTO->setEnabled(true);
        to_horizontal_mode = "Hold_Att";
    }

}

void MainWindow::on_PositionModeTO_clicked()
{
    bool checked = ui->PositionModeTO->isChecked();
    if(checked){
        ui->RollValTO->setEnabled(false);
        ui->PitchValTO->setEnabled(false);
        to_horizontal_mode = "Hold_Pos";
    }
}

void MainWindow::on_HeadingModeFreeOT_clicked()
{
    to_heading_mode = "Free";
}

void MainWindow::on_HeadingModeHoldAngleOT_clicked()
{
    to_heading_mode = "Hold_Angle";
}

void MainWindow::on_HeadingModeZeroRateOT_clicked()
{
    to_heading_mode = "Zero_Rate";
}

void MainWindow::on_ZeroVeloL_clicked()
{
    l_horizontal_mode = "ZeroVelo";

    ui->KnownPoseLX->setEnabled(false);
    ui->KnownPoseLY->setEnabled(false);
    ui->RollValL->setEnabled(false);
    ui->PitchValL->setEnabled(false);
    ui->HMarkerL->setEnabled(false);
    ui->HMarkerDir->setEnabled(false);
    ui->HMarkerLBrowse->setEnabled(false);
    ui->ObjectTrackingL->setEnabled(false);
    ui->ObjectTrackingLBrowse->setEnabled(false);
    ui->ObjectTrackingLDir->setEnabled(false);
    ui->CamAngL->setEnabled(false);
    ui->ObjTrackingCamAngle->setEnabled(false);
    ui->ColorTrackingL->setEnabled(false);
    ui->ColorTrackingLDir->setEnabled(false);
    ui->ColorTrackingLBrowse->setEnabled(false);
}

void MainWindow::on_CurrentPoseL_clicked()
{
    l_horizontal_mode = "CurrPos";

    ui->KnownPoseLX->setEnabled(false);
    ui->KnownPoseLY->setEnabled(false);
    ui->RollValL->setEnabled(false);
    ui->PitchValL->setEnabled(false);
    ui->HMarkerL->setEnabled(false);
    ui->HMarkerDir->setEnabled(false);
    ui->HMarkerLBrowse->setEnabled(false);
    ui->ObjectTrackingL->setEnabled(false);
    ui->ObjectTrackingLBrowse->setEnabled(false);
    ui->ObjectTrackingLDir->setEnabled(false);
    ui->CamAngL->setEnabled(false);
    ui->ObjTrackingCamAngle->setEnabled(false);
    ui->ColorTrackingL->setEnabled(false);
    ui->ColorTrackingLDir->setEnabled(false);
    ui->ColorTrackingLBrowse->setEnabled(false);

}

void MainWindow::on_KnownPoseL_clicked()
{
    l_horizontal_mode = "KnownPos";

    ui->KnownPoseLX->setEnabled(true);
    ui->KnownPoseLY->setEnabled(true);
    ui->RollValL->setEnabled(false);
    ui->PitchValL->setEnabled(false);
    ui->HMarkerL->setEnabled(false);
    ui->HMarkerDir->setEnabled(false);
    ui->HMarkerLBrowse->setEnabled(false);
    ui->ObjectTrackingL->setEnabled(false);
    ui->ObjectTrackingLBrowse->setEnabled(false);
    ui->ObjectTrackingLDir->setEnabled(false);
    ui->CamAngL->setEnabled(false);
    ui->ObjTrackingCamAngle->setEnabled(false);
    ui->ColorTrackingL->setEnabled(false);
    ui->ColorTrackingLDir->setEnabled(false);
    ui->ColorTrackingLBrowse->setEnabled(false);

}

void MainWindow::on_HoldAttL_clicked()
{
    l_horizontal_mode = "HoldAtt";

    ui->RollValL->setEnabled(true);
    ui->PitchValL->setEnabled(true);
    ui->KnownPoseLX->setEnabled(false);
    ui->KnownPoseLY->setEnabled(false);
    ui->HMarkerL->setEnabled(false);
    ui->HMarkerDir->setEnabled(false);
    ui->HMarkerLBrowse->setEnabled(false);
    ui->ObjectTrackingL->setEnabled(false);
    ui->ObjectTrackingLBrowse->setEnabled(false);
    ui->ObjectTrackingLDir->setEnabled(false);
    ui->CamAngL->setEnabled(false);
    ui->ObjTrackingCamAngle->setEnabled(false);
    ui->ColorTrackingL->setEnabled(false);
    ui->ColorTrackingLDir->setEnabled(false);
    ui->ColorTrackingLBrowse->setEnabled(false);

}

void MainWindow::on_PrecisionLandingL_clicked()
{
    l_horizontal_mode = "Prec";

    ui->KnownPoseLX->setEnabled(false);
    ui->KnownPoseLY->setEnabled(false);
    ui->RollValL->setEnabled(false);
    ui->PitchValL->setEnabled(false);
    ui->HMarkerL->setEnabled(true);
    ui->HMarkerDir->setEnabled(true);
    ui->HMarkerLBrowse->setEnabled(true);
    ui->ObjectTrackingL->setEnabled(true);
    ui->ObjectTrackingLBrowse->setEnabled(true);
    ui->ObjectTrackingLDir->setEnabled(true);
    ui->CamAngL->setEnabled(true);
    ui->ObjTrackingCamAngle->setEnabled(true);
    ui->ColorTrackingL->setEnabled(true);
    ui->ColorTrackingLDir->setEnabled(true);
    ui->ColorTrackingLBrowse->setEnabled(true);

}

void MainWindow::on_CurrentAngleHeadingModeL_clicked()
{
    l_heading_mode = "CurrAng";
}

void MainWindow::on_FreeHeadingModeL_clicked()
{
    l_heading_mode = "Free";
}

void MainWindow::on_ObjectTrackingLBrowse_clicked()
{
    QString str = QFileDialog::getOpenFileName(this,"Loading Object Tracking settings");
    ui->ObjectTrackingLDir->setText(str.toUtf8().constData());
}

void MainWindow::on_ColorTrackingLBrowse_clicked()
{
    QString str = QFileDialog::getOpenFileName(this,"Loading Color Tracking settings");
    ui->ColorTrackingLDir->setText(str.toUtf8().constData());
}

void MainWindow::on_HeadingModeAnglePH_clicked()
{
    ph_heading_mode = "Ang";
    ui->AngleValPH->setEnabled(true);
    ui->RateValPH->setEnabled(false);
}

void MainWindow::on_HeadingModeRatePH_clicked()
{
    ph_heading_mode = "Rate";
    ui->AngleValPH->setEnabled(false);
    ui->RateValPH->setEnabled(true);
}

void MainWindow::on_HeadingModeHoldAnglePH_clicked()
{
    ph_heading_mode = "HoldAng";
    ui->AngleValPH->setEnabled(false);
    ui->RateValPH->setEnabled(false);
}

void MainWindow::on_StopConditionTurnPH_clicked()
{
    ph_stop_condition = "Turn";
    ui->TurnsValPH->setEnabled(true);
    ui->TimeValPH->setEnabled(false);
    ui->LimitAnglePH->setEnabled(false);
}

void MainWindow::on_StopConditionTimePH_clicked()
{
    ph_stop_condition = "Time";
    ui->TurnsValPH->setEnabled(false);
    ui->TimeValPH->setEnabled(true);
    ui->LimitAnglePH->setEnabled(false);
}

void MainWindow::on_StopConditionLimitAnglePH_clicked()
{
    ph_stop_condition = "LimitAng";
    ui->TurnsValPH->setEnabled(false);
    ui->TimeValPH->setEnabled(false);
    ui->LimitAnglePH->setEnabled(true);
}

void MainWindow::on_SavePH_clicked()
{
    WritePH2YAML();
}

void MainWindow::on_SaveAsPH_clicked()
{
    QString str = QFileDialog::getSaveFileName(this,"Saving settings");
    position_hold_dir = str.toUtf8().constData();
    WritePH2YAML();
}

void MainWindow::on_UploadAndSave_clicked()
{
    //QString str = QFileDialog::getSaveFileName(this,"Saving Flight Plan");
    QString str = "/home/hojat/GS_WS/src/Ground_Station_1-build/devel/lib/Ground_Station_1/flight_plan.yaml";

    QFile fplan(str);
    fplan.open(QIODevice::ReadWrite);
    QTextStream ts(&fplan);
    ts << ui->AutonomousPlain->toPlainText();
    fplan.flush();
    fplan.close();

    send_flight_plan();

    //emit UploadSignal()
}

void MainWindow::on_ClearFlightPlan_clicked()
{
    ui->AutonomousPlain->clear();
    ui->AutonomousPlain->moveCursor(QTextCursor::End);
    ui->AutonomousPlain->insertPlainText("%YAML:1.0\n\n#Flight plan blocks:");
    block_number = 1;
}

void MainWindow::on_BrowseHDetection_clicked()
{
    QString str = QFileDialog::getOpenFileName(this,"Loading Flight Plan settings");
    ui->HDetectionDir->setText(str.toUtf8().constData());
}

void MainWindow::on_StartHDetection_clicked()
{
    tracking_tab = "h";
    if(ui->DestinationName_H->text().toUtf8().constData() != "")
        destination_name_h = ui->DestinationName_H->text().toUtf8().constData() ;
    emit DestinationNameSignal(&destination_name_h);

    String dir = ui->HDetectionDir->text().toUtf8().constData();
    emit DirSignal(&dir);
}

void MainWindow::on_StopHDetection_clicked()
{
    tracking_tab = "obj";
    emit StopLoadedTrack(true);
}

void MainWindow::on_StartColorTacking_clicked()
{
    emit HSVSignal(ui->h_minTrackbar->value(),ui->h_maxTrackbar->value(),ui->s_minTrackbar->value(),
                   ui->s_maxTrackbar->value(),ui->v_minTrackbar->value(),ui->v_maxTrackbar->value());
}

void MainWindow::on_StopColorTracking_clicked()
{
    emit StopColorTrackingSignal(true);
}

void MainWindow::on_h_minTrackbar_valueChanged(int value)
{
    emit HSVSignal(ui->h_minTrackbar->value(),ui->h_maxTrackbar->value(),ui->s_minTrackbar->value(),
                   ui->s_maxTrackbar->value(),ui->v_minTrackbar->value(),ui->v_maxTrackbar->value());
}

void MainWindow::on_h_maxTrackbar_valueChanged(int value)
{
    emit HSVSignal(ui->h_minTrackbar->value(),ui->h_maxTrackbar->value(),ui->s_minTrackbar->value(),
                   ui->s_maxTrackbar->value(),ui->v_minTrackbar->value(),ui->v_maxTrackbar->value());
}

void MainWindow::on_s_minTrackbar_valueChanged(int value)
{
    emit HSVSignal(ui->h_minTrackbar->value(),ui->h_maxTrackbar->value(),ui->s_minTrackbar->value(),
                   ui->s_maxTrackbar->value(),ui->v_minTrackbar->value(),ui->v_maxTrackbar->value());
}

void MainWindow::on_s_maxTrackbar_valueChanged(int value)
{
    emit HSVSignal(ui->h_minTrackbar->value(),ui->h_maxTrackbar->value(),ui->s_minTrackbar->value(),
                   ui->s_maxTrackbar->value(),ui->v_minTrackbar->value(),ui->v_maxTrackbar->value());
}

void MainWindow::on_v_minTrackbar_valueChanged(int value)
{
    emit HSVSignal(ui->h_minTrackbar->value(),ui->h_maxTrackbar->value(),ui->s_minTrackbar->value(),
                   ui->s_maxTrackbar->value(),ui->v_minTrackbar->value(),ui->v_maxTrackbar->value());
}

void MainWindow::on_v_maxTrackbar_valueChanged(int value)
{
    emit HSVSignal(ui->h_minTrackbar->value(),ui->h_maxTrackbar->value(),ui->s_minTrackbar->value(),
                   ui->s_maxTrackbar->value(),ui->v_minTrackbar->value(),ui->v_maxTrackbar->value());
}

void MainWindow::on_ZeroVeloPH_clicked()
{
    ph_horizontal_mode = "ZeroVelo";
}

void MainWindow::on_CurrPosPH_clicked()
{
    ph_horizontal_mode = "CurrPos";
}

void MainWindow::on_UploadGains_clicked()
{
    emit PIDGainsSignal(ui->PID_X_P_VAL->value(), ui->PID_X_I_VAL->value(), ui->PID_X_D_VAL->value(),
                        ui->PID_Y_P_VAL->value(), ui->PID_Y_I_VAL->value(), ui->PID_Y_D_VAL->value(),
                        ui->PID_Z_P_VAL->value(), ui->PID_Z_I_VAL->value(), ui->PID_Z_D_VAL->value(),
                        ui->PID_V_X_P_VAL->value(), ui->PID_V_X_I_VAL->value(), ui->PID_V_X_D_VAL->value(),
                        ui->PID_V_Y_P_VAL->value(), ui->PID_V_Y_I_VAL->value(), ui->PID_V_Y_D_VAL->value(),
                        ui->PID_V_Z_P_VAL->value(), ui->PID_V_Z_I_VAL->value(), ui->PID_V_Z_D_VAL->value(),
                        ui->PID_H_P_VAL->value(), ui->PID_H_I_VAL->value(), ui->PID_H_D_VAL->value(),
                        ui->PID_YAW_RATE_P_VAL->value(), ui->PID_YAW_RATE_I_VAL->value(), ui->PID_YAW_RATE_D_VAL->value());
}

void MainWindow::on_CancelCurrBlockSemi_clicked()
{
    string command = "cancel";
    emit CommandSemiSignal(&command);
}

void MainWindow::on_ContinueSemi_clicked()
{
    string command = "continue";
    emit CommandSemiSignal(&command);
}

void MainWindow::on_ShutDownSemi_clicked()
{
    string command = "shut_down";
    emit CommandSemiSignal(&command);
}

void MainWindow::on_StartFull_clicked()
{
    string command = "start";
    emit CommandFullSignal(&command);
}

void MainWindow::on_CancelCurrBlockFull_clicked()
{
    string command = "cancel";
    emit CommandFullSignal(&command);
}


void MainWindow::on_ContinueFull_clicked()
{
    string command = "continue";
    emit CommandFullSignal(&command);
}

void MainWindow::on_StandByFull_clicked()
{
    string command = "stand_by";
    emit CommandFullSignal(&command);
}

void MainWindow::on_ShutDownFull_clicked()
{
    string command = "shut_down";
    emit CommandFullSignal(&command);
}

void MainWindow::on_EnableFull_clicked()
{
    if(ui->EnableFull->isChecked()){
        ui->EnableSemi->setChecked(false);
        ui->BlocksManagementSemi->setEnabled(false);
        ui->CancelCurrBlockSemi->setEnabled(false);
        ui->ContinueSemi->setEnabled(false);
        ui->ShutDownSemi->setEnabled(false);

        ui->BlocksManagementFull->setEnabled(true);
        ui->StartFull->setEnabled(true);
        ui->CancelCurrBlockFull->setEnabled(true);
        ui->ContinueFull->setEnabled(true);
        ui->StandByFull->setEnabled(true);
        ui->ShutDownFull->setEnabled(true);
    }
    else{
        ui->BlocksManagementFull->setEnabled(false);
        ui->StartFull->setEnabled(false);
        ui->CancelCurrBlockFull->setEnabled(false);
        ui->ContinueFull->setEnabled(false);
        ui->StandByFull->setEnabled(false);
        ui->ShutDownFull->setEnabled(false);
    }
}

void MainWindow::on_EnableSemi_clicked()
{
    if(ui->EnableSemi->isChecked()){
        ui->EnableFull->setChecked(false);
        ui->BlocksManagementFull->setEnabled(false);
        ui->StartFull->setEnabled(false);
        ui->CancelCurrBlockFull->setEnabled(false);
        ui->ContinueFull->setEnabled(false);
        ui->StandByFull->setEnabled(false);
        ui->ShutDownFull->setEnabled(false);

        ui->BlocksManagementSemi->setEnabled(true);
        ui->CancelCurrBlockSemi->setEnabled(true);
        ui->ContinueSemi->setEnabled(true);
        ui->ShutDownSemi->setEnabled(true);
    }
    else{
        ui->BlocksManagementSemi->setEnabled(false);
        ui->CancelCurrBlockSemi->setEnabled(false);
        ui->ContinueSemi->setEnabled(false);
        ui->ShutDownSemi->setEnabled(false);
    }
}
