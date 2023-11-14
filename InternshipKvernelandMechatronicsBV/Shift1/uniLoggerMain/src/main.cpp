//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        25-09-2023
// Project:     Unilog
// Filename:    main.cpp
// Changed:     30-10-2023
//
// Discription: This is a sourcefile for the Unilog project.
//              It creates a in line comment application to control within a terminal. And runs the specific control function.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header includes
#include "cameraControl.h"
#include "GLViewer.hpp"

// MISC includes
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <string>
#include <utility>
#include <fstream>
#include <string_view>
#include <string>

// Vision includes
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <cuda_runtime.h>
#include <cuda.h>
#include <cuda_gl_interop.h>

// Sample includes
#include <thread>
#include <mutex>


// using namespace
using namespace sl;
using namespace std;


// bools to control functions
bool beginRecord = false;
bool beginPlayback = false;
bool getSerialNumber = false;
bool getImage = false;
bool getDepth = false;
bool getSensordata = false;
bool beginSvoToPcl = false;
bool beginTrackPosition = false;
bool beginSpatialMapping = false;
bool beginTestWindow = false;
bool beginPclFilter = false;
bool beginPclViewer = false;
bool beginPclFilterCompare = false;



// Create a ZED camera called unilog.
cameraControl unilog; 


// main loop
int main(int argc, char **argv) //  | argv[0]: run command | argv[1]: Function | argv[2]: SVO path |
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     Argument Count checker     //////////////////////////////////////////////////////////////////////////
    
    switch (argc)
    {
        // When one argument is given: Always return To less arguments and exit program.
        case 1:
            cout << "[Usage] To less arguments (1). " << endl;
            cout << "[Usage] Type -h or --help for functions." << endl;
            return EXIT_FAILURE;
            break;

        // When two arguments are given and SVO function: Return "To less arguments" and exit program. (SVO functions need a path as third argument.)
        // Else: run command.
        case 2:
            if (string(argv[1]) == "-r" || string(argv[1]) == "--record" || string(argv[1]) == "-p" || string(argv[1]) == "--play" || string(argv[1]) == "-stp" || string(argv[1]) == "--svoToPcl" || string(argv[1]) == "-sm" || string(argv[1]) == "--spatialMapping" || string(argv[1]) == "-pv" || string(argv[1]) == "--pclViewer" || string(argv[1]) == "-pf" || string(argv[1]) == "--pclFilter" || string(argv[1]) == "-pfc" || string(argv[1]) == "--pclfilterCompare")
            {
                cout << "[Usage] To less arguments (2). " << endl;
                cout << "[Usage] SVOPath should be given as third argument." << endl;
                cout << "[Usage] Type -h or --help for functions." << endl;
                return EXIT_FAILURE;
            }
            break;

        // When three arguments are given and SVO function: run command. (SVO functions need a path as third argument.)
        // Else: return "to many arguments and exit program."
        case 3:
            if (string(argv[1]) == "-r" || string(argv[1]) == "--record" || string(argv[1]) == "-p" || string(argv[1]) == "--play" || string(argv[1]) == "-stp" || string(argv[1]) == "--svoToPcl" || string(argv[1]) == "-sm" || string(argv[1]) == "--spatialMapping" || string(argv[1]) == "-pv" || string(argv[1]) == "--pclViewer" || string(argv[1]) == "-pf" || string(argv[1]) == "--pclFilter" || string(argv[1]) == "-pfc" || string(argv[1]) == "--pclfilterCompare")
            {
                break;
            }
            else
            {
                cout << "[Usage] To many arguments (2). " << endl;
                cout << "[Usage] This function does not take three argumets." << endl;
                cout << "[Usage] Type -h or --help for functions." << endl;
                return EXIT_FAILURE;
            }
            break;

        // When four - nine arguments are given: Always return "to may arguments" and exit program.
        case 4 ... 9:
            cout << "[Usage] To many arguments." << endl;
            cout << "[Usage] Type -h or --help for functions" << endl;
            return EXIT_FAILURE;
            break;

        // When there is an different format given: return "no valid format" and exit program.
        default:
            cout << "[Usage] No valid format." << endl;
            cout << "[Usage] Type -h or --help for functions" << endl;
            return EXIT_FAILURE;
            break;
    }
    

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     Argument Value checker     //////////////////////////////////////////////////////////////////////////

    if (string(argv[1]) == "-h" || string(argv[1]) == "--help")
    {
        unilog.help(); // print the help function
    }
    else if (string(argv[1]) == "-r" || string(argv[1]) == "--record") // record svo
    {
        cout << "\n[Dummy] In function record" << endl;
        beginRecord = true;
    }
    else if (string(argv[1]) == "-p" || string(argv[1]) == "--play") // play svo
    {
        cout << "\n[Dummy] In function playback" << endl;
        beginPlayback = true;
    }
    else if (string(argv[1]) == "-gs" || string(argv[1]) == "--getSerialnumber") // get serialnumber
    {
        cout << "\n[Dummy] In function getSerialnumber" << endl;
        getSerialNumber = true;
    }
        else if (string(argv[1]) == "-gi" || string(argv[1]) == "--getImage") // get image data
    {
        cout << "\n[Dummy] In function getImage" << endl;
        getImage = true;
    }
        else if (string(argv[1]) == "-gd" || string(argv[1]) == "--getDepth") // get depth data
    {
        cout << "\n[Dummy] In function getDepth" << endl;
        getDepth = true;
    }
    else if (string(argv[1]) == "-gsd" || string(argv[1]) == "--getSensordata") // get sensor data
    {
        cout << "\n[Dummy] In function getSensordata" << endl;
        getSensordata = true;
    }
    else if (string(argv[1]) == "-stp" || string(argv[1]) == "--svoToPcl") // svo to pcl pointcloud
    {
        cout << "\n[Dummy] In function svo to pcl" << endl;
        beginSvoToPcl = true;
    }
    else if (string(argv[1]) == "-tp" || string(argv[1]) == "--trackPosition") // track camera position
    {
        cout << "\n[Dummy] In function trackPosition" << endl;
        beginTrackPosition = true;
    }
    else if (string(argv[1]) == "-sm" || string(argv[1]) == "--spatialMapping") // spatial mapping
    {
        cout << "\n[Dummy] In function spatialMapping" << endl;
        beginSpatialMapping = true;
    }
    else if (string(argv[1]) == "-tw" || string(argv[1]) == "--testWindow") // test Window
    {
        cout << "\n[Dummy] In function testWindow" << endl;
        beginTestWindow = true;
    }
    else if (string(argv[1]) == "-pf" || string(argv[1]) == "--pclFilter") // filter single pointcloud
    {
        cout << "\n[Dummy] In function filterPointcloud" << endl;
        beginPclFilter = true;
    }
    else if (string(argv[1]) == "-pv" || string(argv[1]) == "--pclViewer") // viewer single pointcloud
    {
        cout << "\n[Dummy] In function pclViewer" << endl;
        beginPclViewer = true;
    }
    else if (string(argv[1]) == "-pfc" || string(argv[1]) == "--pclfilterCompare") // viewer single pointcloud
    {
        cout << "\n[Dummy] In function pclViewer" << endl;
        beginPclFilterCompare = true;
    }
    else
    {
        cout << endl;
        cout << "[Usage] unvalid command." << endl;
        cout << "[Usage] Type -h or --help for functions" << endl;
        exit(0);
    }
    

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     Main Code   /////////////////////////////////////////////////////////////////////////////////////////


    // initialize the ZED camera.
    unilog.initCamera(); 
    

    if (beginRecord)
    {
        unilog.initRecord(argv[2]); // initialize recording params.
        unilog.openCamera(); // Open the ZED camera.

        if(unilog.recordSVO()) // start SVO record and return true when done.
        {
            cout << "[Camera Recording] SVO recorded." << endl;
        }
        // cout << "\n[Dummy] in the if statement for beginning the record" << endl;
    }


    if (beginPlayback)
    {
        unilog.initSVO(argv[2]); // initialze svo params
        unilog.openCamera(); // Open the ZED camera.

        if (unilog.playbackSVO()) // start SVO playback and return true when done.
        {
            cout << "[SVO Playback] SVO playback done." << endl;
        }
        // cout << "\n[Dummy] in the if statement for beginning the playback" << endl;
    }


    if (beginSvoToPcl)
    {
        unilog.initSVO(argv[2]); // initialze svo params
        unilog.openCamera(); // Open the ZED camera.

        if (unilog.svoToPCL()) // start SVO to PCL transformation and return true when done.
        {
            cout << "[SVO t PCL] SVO to PCL transformation done." << endl;
        }
        // cout << "\n[Dummy] in the if statement for beginning the playback" << endl;
    }
    

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     Side functions   /////////////////////////////////////////////////////////////////////////////////////////

    if (beginTrackPosition)
    {
        unilog.openCamera(); // Open the ZED camera.

        if(unilog.trackPosition()) // start positional tracking and return true when done.
        {
            cout << "[Camera Tracking] Position tracked." << endl;
        }
        // cout << "\n[Dummy] in the if statement for beginning positional tracking" << endl;
    }


    if (beginSpatialMapping)
    {
        unilog.openCamera(); // Open the ZED camera.

        if(unilog.createSpatialMap(argv[2], 300)) // start spatial mapping and return true when done.
        {
            cout << "[Camera Mapping] Spatial map created." << endl;
        }
        // cout << "\n[Dummy] in the if statement for spatial mapping." << endl;
    }


    if (beginTestWindow)
    {
        unilog.openCamera(); // Open the ZED camera.

        if(unilog.testWindow()) // start testwindow and return true when done.
        {
            cout << "[Test Window] Test window done." << endl;
        }
        // cout << "\n[Dummy] in the if statement for test window." << endl;
    }


    if (beginPclFilter)
    {
        if(unilog.pclFilter(argv[2])) // start pcl filter and return true when done.
        {
            cout << "[PCL Filter] Filter done." << endl;
        }
        // cout << "\n[Dummy] in the if statement for filter pcl." << endl;
    }
    
    
    if (beginPclViewer)
    {
        if(unilog.pclViewer(argv[2])) // start pcl viewer and return true when done.
        {
            cout << "[PCL Viewer] Viewer done." << endl;
        }
        // cout << "\n[Dummy] in the if statement for view pcl." << endl;
    }


    if (beginPclFilterCompare)
    {
        if(unilog.pclFilterCompare(argv[2])) // start pcl filter compare and return true when done.
        {
            cout << "[PCL Viewer] Compare done." << endl;
        }
        // cout << "\n[Dummy] in the if statement for view pcl." << endl;
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     Get Information Functions   /////////////////////////////////////////////////////////////////////////

    if (getSerialNumber)
    {
        unilog.openCamera(); // Open the ZED camera.
        
        if(unilog.getSerialNumber()) // get Serialnumber and return true when done.
        {
            cout << "[Camera get] Serialnumber received." << endl;
        }
        //cout << "\n[Dummy] in the if statement for getting serialnumber" << endl;
    }


    if (getImage)
    {
        unilog.openCamera(); // Open the ZED camera.
        
        if(unilog.getImage()) // get Image data and return true when done.
        {
            cout << "[Camera get] Image data received." << endl;
        }
        //cout << "\n[Dummy] in the if statement for getting Image" << endl;
    }


    if (getDepth)
    {
        unilog.openCamera(); // Open the ZED camera.
        
        if(unilog.getDepth()) // get Image data and return true when done.
        {
            cout << "[Camera get] Depth data received." << endl;
        }
        //cout << "\n[Dummy] in the if statement for getting depth" << endl;
    }


    if (getSensordata)
    {
        unilog.openCamera(); // Open the ZED camera.
        
        if(unilog.getSensordata()) // get Image data and return true when done.
        {
            cout << "[Camera get] Sensor data received." << endl;
        }
        //cout << "\n[Dummy] in the if statement for getting sensor data" << endl;
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     Closing the Code   //////////////////////////////////////////////////////////////////////////////////


    unilog.closeCamera(); // Close the camera

}
    

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     The End :)     //////////////////////////////////////////////////////////////////////////////////////
    
        