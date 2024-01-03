#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger software
// Filename:    cameraController.h
// Changed:     03-01-2023 | 14:35
//
// Discription: this file contains the class to control the zed stereo camera from stereolabs
//              Specify


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Camera Includes
#include <sl/Camera.hpp>

// MISC Includes
#include <utility>
#include <iostream>
#include <stdio.h>
#include <ctime>
#include <string>
#include <fstream>

// Using namespaces
using namespace sl;
using namespace std;

extern Camera zed;
extern Mat slPointcloud;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Class: cameraController      /////////////////////////////////////////////////////////////////////

// class to control ZED stereocamera
class cameraController
{
    private:

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

        

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///     Setup and init     //////////////////////////////////////////////////////////////////////////////////

        bool openCamera(void); 
        bool initCamera(void);
        bool initSVO(char *filePathSVO);
        bool initRecord(char *filePathSVO);
        bool closeCamera(void);
        bool getSlPointcloud(void);
        
        bool enableMapping(void);
        bool disableMapping(void);
        bool enableTracking(void);
        bool disableTracking(void);
};


#endif // CAMERA_CONTROLLER_H


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////