#ifndef CAMERA_CONTROL_H
#define CAMERA_CONTROL_



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        25-09-2023
// Project:     Unilog
// Filename:    cameraControl.h
// Changed:     30-10-2023
// Discription: This is a headerfile for the cameraControl class tha inheritens from the ZED SDK Camera class.
//              It controls a serie functions to create a application for the Unilog project.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Vision includes
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

// MISC includes
#include <iostream>
#include <stdio.h>

// PCL includes
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

// Sample includes
#include <thread>
#include <mutex>

// Namespaces
using namespace sl;
using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Camera Control Class     ////////////////////////////////////////////////////////////////////////////

// Class to Control ZED Camera.
class cameraControl: public Camera
{
private:
    
    // Need to check wich functions can go to the private members.

public:
    

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     Parameters     //////////////////////////////////////////////////////////////////////////////////////
    
    // for camera configration
    InitParameters initParams; // Params to initialize camera.
    RuntimeParameters runtimeParams; // Params to initialize video.
    PositionalTrackingParameters trackingParams; // Params to initialize positional tracking.
    SpatialMappingParameters mappingParams; // Params to initialize spatial mapping
    Pose zedPose; // create position from Pose class

    // for recording svo
    RecordingParameters recordingParams; // Params to initialize recording.
    RecordingStatus recStatus; // Status to keep track of the recording.

    // for svo to pcl pointcloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointcloud; // pointcloud in PCL format
    
   

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     Setup and Init     //////////////////////////////////////////////////////////////////////////////////

    void help(void);
    bool openCamera(void); 
    bool initCamera(void);
    bool initSVO(char *filePathSVO);
    bool initRecord(char *filePathSVO);
    bool closeCamera(void);
    
    bool enableTracking(void);
    bool enableMapping(void);

 
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     Main functions     //////////////////////////////////////////////////////////////////////////////////

    bool recordSVO(void);
    bool playbackSVO(void);
    bool svoToPCL(void);

    void startThread(void);
    void runThread(void);
    void stopThread(void);


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     Side Functions     //////////////////////////////////////////////////////////////////////////////////

    bool pclFilter(char *filePath);
    bool pclViewer(char *filePath);
    bool pclFilterCompare(char* filepath);
    bool trackPosition(int count = 500);
    bool createSpatialMap(char *filePath, int count= 500);
    
    bool testWindow(void);


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     Info Functions     //////////////////////////////////////////////////////////////////////////////////
    
    bool getSerialNumber(void);
    bool getImage(int count = 50);
    bool getDepth(int count = 50);
    bool getSensordata(void);
    bool printSensorData(void);
};


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     The End :)     //////////////////////////////////////////////////////////////////////////////////////   



#endif // CAMERA_CONTROL_H



