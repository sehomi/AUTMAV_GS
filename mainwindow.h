#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QThread>
#include <QtCore>
#include <QtGui>
#include <QString>
#include <QToolButton>

#include "roshandler.h"
#include "TakeOffMsg.h"
#include "LandingMsg.h"
#include "ColorTrackingMsg.h"
#include "ObjectTrackingMsg.h"

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    void WriteOT2YAML();
    void WriteCT2YAML();
    void WriteTO2YAML();
    void WriteL2YAML();
    void WritePH2YAML();
    void WriteHD2YAML();
    void addCommand();
    void sendCommandFull(QToolButton*);
    void sendCommandSemi(QToolButton*);

    void send_flight_plan();

    RosHandler *rh;
    QThread *mainThread;

    string obj_tracking_dir = "Object_Tracking.yaml";
    string color_tracking_dir = "Color_Tracking.yaml";
    string take_off_dir = "Take_Off.yaml";
    string landing_dir = "Landing.yaml";
    string position_hold_dir = "Position_Hold.yaml";
    string h_detection_dir = "H_Detection.yaml";

    string destination_name = "object.png";
    string destination_name_h = "hmarker.png";
    string glob_method = "Surf";
    string to_horizontal_mode = "Hold_Pos";
    string to_heading_mode = "Zero_Rate";
    string l_horizontal_mode = "CurrPos";
    string l_heading_mode = "CurrAng";
    string ph_horizontal_mode = "ZeroVelo";
    string ph_heading_mode = "HoldAng";
    string ph_stop_condition = "Turn";

    vector <pair<string,bool> > take_off_settings;
    vector <pair<string,bool> > landing_settings;
    vector <pair<string,bool> > position_hold_settings;

    string tracking_tab = "obj";

    bool last_ls = true;
    bool last_tos = true;
    bool last_phs = true;

    int block_number = 1;

    vector <pair<string ,pair<int,vector <QToolButton*> > > > blocks;

    QVBoxLayout *layout;

signals:
    void TrackSignal(bool);
    void MethodSignal(String*);
    void HSVSignal(int,int,int,int,int,int);
    void StopColorTrackingSignal(bool);
    void DirSignal(String*);
    void StopLoadedTrack(bool);
    void UploadSignal(string *);
    void DestinationNameSignal(string*);
    void PIDGainsSignal(int,int,int,
                        int,int,int,
                        int,int,int,
                        int,int,int,
                        int,int,int,
                        int,int,int,
                        int,int,int,
                        int,int,int);
    void BlockCommandFull(int);
    void BlockCommandSemi(int);
    void CommandFullSignal(string *);
    void CommandSemiSignal(string *);

public slots:
    void onObjectTrackingImgSignal(Mat*);
    void onColorTrackingImgSignal(Mat*);
    void onColorTrackingThreshImgSignal(Mat*);
    void onGridSignal(vector <float> *);
    void onCommandsSignal(float, float, float, float, bool);
    void onNavigationSignal(float, float, float,
                            float, float, float,
                            float, float, float,
                            float, float, float);
    void onStringMsgSignal(string*);

private slots:
    void on_GrabObjAndStart_clicked();
    void on_StopTracking_clicked();
    void on_LoadObjectCheckbox_stateChanged(int arg1);
    void on_GrabImgCheckbox_stateChanged(int arg1);
    void on_LoadObjectBrowse_clicked();
    void on_LoadObjectAndStart_clicked();
    void on_LoadStopTracking_clicked();
    void on_SaveAsObjTracking_clicked();
    void on_SaveObjTracking_clicked();
    void on_SurfCheckbox_clicked();
    void on_SiftCheckbox_clicked();
    void on_OrbCheckbox_clicked();
    void on_FASTOrbCheckBox_clicked();
    void on_SaveAsColorTracking_clicked();
    void on_SaveColorTracking_clicked();
    void on_SaveAsTakeOff_clicked();
    void on_SaveTakeOff_clicked();
    void on_SaveAsLanding_clicked();
    void on_SaveLanding_clicked();
    void on_BrowseTeakeOff_clicked();
    void on_BrowseLanding_clicked();
    void on_BrowseBeams_clicked();
    void on_PositionHoldBrowse_clicked();
    void on_AddTakeOff_clicked();
    void on_AddLanding_clicked();
    void on_AddBeams_clicked();
    void on_AddPositionHold_clicked();
    void on_ToolButtons_clicked_full();
    void on_ToolButtons_clicked_semi();
    void on_AttidtudeModeTO_clicked();
    void on_PositionModeTO_clicked();
    void on_HeadingModeFreeOT_clicked();
    void on_HeadingModeHoldAngleOT_clicked();
    void on_HeadingModeZeroRateOT_clicked();
    void on_CurrentPoseL_clicked();
    void on_KnownPoseL_clicked();
    void on_HoldAttL_clicked();
    void on_PrecisionLandingL_clicked();
    void on_CurrentAngleHeadingModeL_clicked();
    void on_FreeHeadingModeL_clicked();
    void on_ObjectTrackingLBrowse_clicked();
    void on_ColorTrackingLBrowse_clicked();
    void on_HeadingModeAnglePH_clicked();
    void on_HeadingModeRatePH_clicked();
    void on_HeadingModeHoldAnglePH_clicked();
    void on_StopConditionTurnPH_clicked();
    void on_StopConditionTimePH_clicked();
    void on_StopConditionLimitAnglePH_clicked();
    void on_SavePH_clicked();
    void on_SaveAsPH_clicked();
    void on_UploadAndSave_clicked();
    void on_ClearFlightPlan_clicked();
    void on_BrowseHDetection_clicked();
    void on_StartHDetection_clicked();
    void on_StopHDetection_clicked();
    void on_StartColorTacking_clicked();
    void on_StopColorTracking_clicked();
    void on_h_minTrackbar_valueChanged(int value);
    void on_h_maxTrackbar_valueChanged(int value);
    void on_s_minTrackbar_valueChanged(int value);
    void on_s_maxTrackbar_valueChanged(int value);
    void on_v_minTrackbar_valueChanged(int value);
    void on_v_maxTrackbar_valueChanged(int value);
    void on_VelocityModeTO_clicked();
    void on_ZeroVeloPH_clicked();
    void on_CurrPosPH_clicked();
    void on_ZeroVeloL_clicked();
    void on_UploadGains_clicked();
    void on_CancelCurrBlockSemi_clicked();
    void on_ShutDownSemi_clicked();
    void on_StartFull_clicked();
    void on_CancelCurrBlockFull_clicked();
    void on_StandByFull_clicked();
    void on_ShutDownFull_clicked();
    void on_EnableFull_clicked();
    void on_EnableSemi_clicked();
    void on_ContinueFull_clicked();
    void on_ContinueSemi_clicked();
};

#endif // MAINWINDOW_H
