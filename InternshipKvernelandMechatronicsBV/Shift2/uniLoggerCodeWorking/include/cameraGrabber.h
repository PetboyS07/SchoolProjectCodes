#ifndef CAMERA_GRABBER_H
#define CAMERA_GRABBER_H

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger Software
// Filename:    cameraGrabber.h
// Changed:     04-01-2024 | 14:44
//
// Discription: This file contains the cameraGrabber class to control all grab functions.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header includes
#include "cameraController.h"
<<<<<<<< HEAD:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerCodeWorking/include/cameraGrabber.h
#include "threadUtilities.hpp"
#include "logCout.h"
========
#include "thread_utilities.hpp"
>>>>>>>> 71cccea26b3de34f671a0b0d3ea02f86ae34e0e1:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerShift2/include/cameraGrabber.h

// MISC includes
#include <stdio.h>
#include <iostream>
#include <thread>
#include <memory>
#include <atomic>
#include <string>
#include <mutex>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Class: cameraGrabber      ///////////////////////////////////////////////////////////////////////////

// class to grab all kind off information from the zed
class cameraGrabber
{
    private:
        
    public:
        void startGrabThread(void);
        void runGrabThread(void);
        void stopGrabThread(void);
        bool transformPointcloud(void);
        inline float convertColor(float colorIn);
        void pushData(void);
};

#endif // CAMERA_GRABBER_H

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////