#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger software
// Filename:    cameraController.h
// Changed:     04-01-2023 | 12:20
//
// Discription: this file contains the class to control the zed stereo camera from stereolabs


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header includes
#include "logCout.h"

// MISC includes
#include <stdio.h>
#include <iostream>
#include <sl/Camera.hpp>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Parameters     //////////////////////////////////////////////////////////////////////////////////////
        
extern sl::Camera zed;
extern sl::Mat slPointcloud;
        
        
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Setup and init     //////////////////////////////////////////////////////////////////////////////////


bool initCamera(void);
bool initSVO(char *filePathSVO);
bool initRecord(char *filePathSVO);
bool openCamera(void); 
void closeCamera(void);
bool getSlPointcloud(void);

bool enableMapping(void);
bool disableMapping(void);
bool enableTracking(void);
bool disableTracking(void);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////

#endif // CAMERA_CONTROLLER_H
