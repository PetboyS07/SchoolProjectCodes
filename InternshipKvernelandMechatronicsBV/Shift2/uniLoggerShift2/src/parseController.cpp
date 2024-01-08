//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger Software
// Filename:    parseController.cpp
// Changed:     03-01-2023 | 14:35
//
// Discription: This file contains all memberfunctions from the parseController class.
//              Specify


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header Includes
#include "parseController.h"
#include "functionController.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Member Functions     ////////////////////////////////////////////////////////////////////////////////


// Constructor
parseController::parseController(void)
{
    beginRecord = false;
    beginPlayback = false;
    getSerialNumber = false;
    getImage = false;
    getDepth = false;
    getSensordata = false;
    beginSvoToPcl = false;
    beginSvoToFilteredPcl = false;
    beginTrackPosition = false;
    beginSpatialMapping = false;
    beginTestWindow = false;
    beginPclFilter = false;
    beginPclViewer = false;
    beginPclFilterCompare = false;  
}


bool parseController::argHelp(void)
{
    cout << endl;
    cout << "[Parser] In function help." << endl;
    cout << endl;
    cout << "[Parser] Command format: ./filename  --function  PathToFile" << endl;
    cout << endl;
    cout << "[Parser] Main Logging Program: " << endl;
    cout << "[Parser] Type '-l' or '--logger' to start the main logger. (svoOutputPath should be given)" << endl;
    cout << endl;
    cout << "[Parser] Main Functions: " << endl;
    cout << "[Parser] Type '-r' or '--record' to record svo. (svoOutputPath should be given)" << endl;
    cout << "[Parser] Type '-p' or '--play' to playback svo. (svoInputPath should be given)" << endl;
    cout << "[Parser] Type '-stp' or '--svoToPcl to transform svo to pcl pointcloud." << endl;
    cout << "[Parser] Type '-stfp' or '--svoToFilteredPCL to transform svo to a filterd pcl pointcloud." << endl;
    cout << endl;
    cout << "[Parser] Side Functions: " << endl;
    cout << "[Parser] Type '-h' or '--help' for a list of commands." << endl;
    cout << "[Parser] Type '-pv' or '--pclViewer' to view pointcloud." << endl;
    cout << "[Parser] Type '-pf' or '--pclFilter' to filter pointcloud." << endl;
    cout << "[Parser] Type '-pfc' or '--pclFilterCompare' to compare filtered pointcloud." << endl;
    cout << "[Parser] Type '-tp' or '--trackPosition to track ZED camera." << endl;    
    cout << "[Parser] Type '-sm' or '--spatialMapping to start spatial mapping." << endl;
    cout << "[Parser] Type '-tw' or '--testWindow to start test window." << endl;    
    cout << endl;
    cout << "[Parser] Get Information Functions: " << endl;
    cout << "[Parser] Type '-gs' or '--getSerialnumber' to get zed serialnumber." << endl;
    cout << "[Parser] Type '-gi' or '--getImage' to get zed image." << endl;
    cout << "[Parser] Type '-gd' or '--getDepth' to get zed depth." << endl;
    cout << "[Parser] Type '-gsd' or '--getSensordata to get sensordata." << endl;
    cout << endl;
    exit(0); // exit the program.

    return true;
}


// This function checks for a valid terminal command.
bool parseController::argCountChecker(int &argc, char *argv1)
{
    switch (argc)
    {
        // When one argument is given: Always return To less arguments and exit program.
        case 1:
            cout << "[Parser] To less arguments (1). " << endl;
            cout << "[Parser] Type -h or --help for functions." << endl;
            exit(0);
            break;

        // When two arguments are given and SVO function: Return "To less arguments" and exit program. (SVO functions need a path as third argument.)
        // Else: run command.
        case 2:
            if (string(argv1) == "-r" || string(argv1) == "--record" || string(argv1) == "-p" || string(argv1) == "--play" || string(argv1) == "-stp" || string(argv1) == "--svoToPcl" || string(argv1) == "-sm" || string(argv1) == "--spatialMapping" || string(argv1) == "-pv" || string(argv1) == "--pclViewer" || string(argv1) == "-pf" || string(argv1) == "--pclFilter" || string(argv1) == "-pfc" || string(argv1) == "--pclfilterCompare" || string(argv1) == "-stfp" || string(argv1) == "--svoToFilteredPcl" || string(argv1) == "-l" || string(argv1) == "--logger")
            {
                cout << "[Parser] To less arguments (2). " << endl;
                cout << "[Parser] SVOPath should be given as third argument." << endl;
                cout << "[Parser] Type -h or --help for functions." << endl;
                exit(0);
            }
            break;

        // When three arguments are given and SVO function: run command. (SVO functions need a path as third argument.)
        // Else: return "to many arguments and exit program."
        case 3:
            if (string(argv1) == "-r" || string(argv1) == "--record" || string(argv1) == "-p" || string(argv1) == "--play" || string(argv1) == "-stp" || string(argv1) == "--svoToPcl" || string(argv1) == "-sm" || string(argv1) == "--spatialMapping" || string(argv1) == "-pv" || string(argv1) == "--pclViewer" || string(argv1) == "-pf" || string(argv1) == "--pclFilter" || string(argv1) == "-pfc" || string(argv1) == "--pclfilterCompare" || string(argv1) == "-stfp" || string(argv1) == "--svoToFilteredPcl" || string(argv1) == "-l" || string(argv1) == "--logger")
            {
                break;
            }
            else
            {
                cout << "[Parser] To many arguments (2). " << endl;
                cout << "[Parser] This function does not take three argumets." << endl;
                cout << "[Parser] Type -h or --help for functions." << endl;
                exit(0);
            }
            break;

        // When four - nine arguments are given: Always return "to may arguments" and exit program.
        case 4 ... 9:
            cout << "[Parser] To many arguments." << endl;
            cout << "[Parser] Type -h or --help for functions" << endl;
            exit(0);
            break;

        // When there is an different format given: return "no valid format" and exit program.
        default:
            cout << "[Parser] No valid format." << endl;
            cout << "[Parser] Type -h or --help for functions" << endl;
            exit(0);
            break;
    }
    return 0;
}


// Functions to read witch function need to be exicuted.
bool parseController::argValueChecker(int &argc, char *argv1)
{
    if (string(argv1) == "-h" || string(argv1) == "--help")
    {
        argHelp(); // print the help function
    }
    else if (string(argv1) == "-r" || string(argv1) == "--record") // record svo
    {
        cout << "\n[Parser] In function record" << endl;
        beginRecord = true;
    }
    else if (string(argv1) == "-p" || string(argv1) == "--play") // play svo
    {
        cout << "\n[Parser] In function playback" << endl;
        beginPlayback = true;
    }
    else if (string(argv1) == "-gs" || string(argv1) == "--getSerialnumber") // get serialnumber
    {
        cout << "\n[Parser] In function getSerialnumber" << endl;
        getSerialNumber = true;
    }
        else if (string(argv1) == "-gi" || string(argv1) == "--getImage") // get image data
    {
        cout << "\n[Parser] In function getImage" << endl;
        getImage = true;
    }
        else if (string(argv1) == "-gd" || string(argv1) == "--getDepth") // get depth data
    {
        cout << "\n[Parser] In function getDepth" << endl;
        getDepth = true;
    }
    else if (string(argv1) == "-gsd" || string(argv1) == "--getSensordata") // get sensor data
    {
        cout << "\n[Parser] In function getSensordata" << endl;
        getSensordata = true;
    }
    else if (string(argv1) == "-stp" || string(argv1) == "--svoToPcl") // svo to pcl pointcloud
    {
        cout << "\n[Parser] In function svo to pcl" << endl;
        beginSvoToPcl = true;
    }
    else if (string(argv1) == "-stfp" || string(argv1) == "--svoToFilteredPcl") // svo to pcl pointcloud
    {
        cout << "\n[Parser] In function svo to a filtered pcl" << endl;
        beginSvoToFilteredPcl = true;
    }
    else if (string(argv1) == "-tp" || string(argv1) == "--trackPosition") // track camera position
    {
        cout << "\n[Parser] In function trackPosition" << endl;
        beginTrackPosition = true;
    }
    else if (string(argv1) == "-sm" || string(argv1) == "--spatialMapping") // spatial mapping
    {
        cout << "\n[Parser] In function spatialMapping" << endl;
        beginSpatialMapping = true;
    }
    else if (string(argv1) == "-tw" || string(argv1) == "--testWindow") // test Window
    {
        cout << "\n[Parser] In function testWindow" << endl;
        beginTestWindow = true;
    }
    else if (string(argv1) == "-pf" || string(argv1) == "--pclFilter") // filter single pointcloud
    {
        cout << "\n[Parser] In function filterPointcloud" << endl;
        beginPclFilter = true;
    }
    else if (string(argv1) == "-pv" || string(argv1) == "--pclViewer") // viewer single pointcloud
    {
        cout << "\n[Parser] In function pclViewer" << endl;
        beginPclViewer = true;
    }
    else if (string(argv1) == "-pfc" || string(argv1) == "--pclfilterCompare") // viewer single pointcloud
    {
        cout << "\n[Parser] In function pclfilterCompare" << endl;
        beginPclFilterCompare = true;
    }
    else if (string(argv1) == "-l" || string(argv1) == "--logger") // viewer single pointcloud
    {
        cout << "\n[Parser] Starting main logger programm" << endl;
    }
    else
    {
        cout << endl;
        cout << "[Parser] unvalid command." << endl;
        cout << "[Parser] Type -h or --help for functions" << endl;
        exit(0);
    }

    return true;
}


bool parseController::runFunction(char *argv2)
{
    //setup function controller.
    functionController unilog;
    unilog.initCamera(); // initialize the ZED camera.

    // Code
    if (beginRecord)
    {
        unilog.initRecord(argv2); // initialize recording params.
        unilog.openCamera(); // Open the ZED camera.

        if(unilog.recordSVO()) // start SVO record and return true when done.
        {
          cout << "[Camera Recording] SVO recorded." << endl;
        }
        cout << "\n[Dummy] in the if statement for beginning the record" << endl;
    }


    if (beginPlayback)
    {
        unilog.initSVO(argv2); // initialze svo params
        unilog.openCamera(); // Open the ZED camera.

        if (unilog.playbackSVO()) // start SVO playback and return true when done.
        {
          cout << "[SVO Playback] SVO playback done." << endl;
        }
        cout << "\n[Dummy] in the if statement for beginning the playback" << endl;
    }


    if (beginSvoToPcl)
    {
        unilog.initSVO(argv2); // initialze svo params
        unilog.openCamera(); // Open the ZED camera.

        if (unilog.svoToPCL()) // start SVO to PCL transformation and return true when done.
        {
            cout << "[SVO to PCL] SVO to PCL transformation done." << endl;
        }
        // cout << "\n[Dummy] in the if statement for beginning the svotofpcl" << endl;
    }
    

    if (beginSvoToFilteredPcl)
    {
        unilog.initSVO(argv2); // initialze svo params
        unilog.openCamera(); // Open the ZED camera.

        if (unilog.svoToFilteredPCL()) // start SVO to PCL transformation and return true when done.
        {
            cout << "[SVO to PCL] SVO to Filtered PCL done." << endl;
        }
        // cout << "\n[Dummy] in the if statement for beginning the svotofilteredpcl" << endl;
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

        if(unilog.createSpatialMap(argv2, 300)) // start spatial mapping and return true when done.
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
        if(unilog.pclFilter(argv2)) // start pcl filter and return true when done.
        {
            cout << "[PCL Filter] Filter done." << endl;
        }
        // cout << "\n[Dummy] in the if statement for filter pcl." << endl;
    }
    
    
    if (beginPclViewer)
    {
        if(unilog.pclViewer(argv2)) // start pcl viewer and return true when done.
        {
            cout << "[PCL Viewer] Viewer done." << endl;
        }
        // cout << "\n[Dummy] in the if statement for view pcl." << endl;
    }


    if (beginPclFilterCompare)
    {
        if(unilog.pclFilterCompare(argv2)) // start pcl filter compare and return true when done.
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
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////