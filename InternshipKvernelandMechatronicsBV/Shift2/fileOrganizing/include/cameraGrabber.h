#ifndef CAMERA_GRABBER_H
#define CAMERA_GRABBER_H


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger Software
// Filename:    cameraGrabber.h
// Changed:     03-01-2023 | 14:35
//
// Discription: This file contains the cameraGrabber class to control all grab functions.
//              Specify


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Camera Includes
#include "threadUtilities.hpp"
#include "cameraController.h"

// MISC Includes
#include <stdio.h>
#include <iostream>
#include <string>

// Sample includes
#include <thread>
#include <mutex>
#include <memory>
#include <atomic>

// Using namespaces
using namespace sl;
using namespace std;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Class: cameraGrabber      ///////////////////////////////////////////////////////////////////////////

// class to grab all kind off information from the zed
class cameraGrabber
{
    private:
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointcloud;
        
    public:
        std::unique_ptr<std::thread> thread;
        std::atomic<bool> runThread;

        //bool stopSignal;
        //bool hasData;
        //mutex mutexInput;
        //Mat slPointcloud;

        cameraController camcon;

        cameraGrabber(void);
        ~cameraGrabber(void);
        void startGrabThread(void);
        void runGrabThread(void);
        void stopGrabThread(void);
        bool transformPointcloud(void);
        inline float convertColor(float colorIn);
        bool pushData(void);

};


#endif // CAMERA_GRABBER_H


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////