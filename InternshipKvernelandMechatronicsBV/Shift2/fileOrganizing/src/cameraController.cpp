//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger software
// Filename:    cameraController.cpp
// Changed:     03-01-2023 | 14:35
//
// Discription: This file contains all memberfunctions from the cameraController class.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header Includes
#include "cameraController.h"

auto returnedState = ERROR_CODE::SUCCESS;
Camera zed;
Mat slPointcloud;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Functions     ///////////////////////////////////////////////////////////////////////////////////////


// Initialyse the camera. 
bool cameraController::initCamera(void) 
{
    cout << endl;
    cout << "[cameraController::initCamera] Begin initializing the ZED camera..." << endl;

	// set init parameters for the camera.
    cout << "[cameraController::initCamera] Set Init Parameters..." << endl;
    returnedState = ERROR_CODE::SUCCESS;
    initParams.sdk_verbose = false; // Enable verbose logging
    initParams.camera_resolution = RESOLUTION::HD1080;
    initParams.camera_fps = 1;
    initParams.depth_mode = DEPTH_MODE::ULTRA;
    initParams.coordinate_units = UNIT::METER; // might be MILIMETER?
    initParams.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    
	// set runtime parameters for the camera.
    cout << "[cameraController::initCamera] Set Runtime Parameters..." << endl;
	// set runtime parameters
	runtimeParams.sensing_mode = sl::SENSING_MODE::STANDARD;
	runtimeParams.enable_depth = true;
    runtimeParams.confidence_threshold = 100;
    runtimeParams.texture_confidence_threshold = 100;
    runtimeParams.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
    runtimeParams.remove_saturated_areas = true;

    // set positional tracking params
    cout << "[cameraController::initCamera] Set Tracking Parameters..." << endl;
    trackingParams.enable_pose_smoothing = true; // corrects small drifts

    // set spetial mapping params
    cout << "[cameraController::initCamera] Set Mapping Parameters..." << endl;
    mappingParams.resolution_meter = 0.03; // set resolution to 3 cm.
    mappingParams.range_meter = 3; // set maximum depth maping range to 3 m.
    mappingParams.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
    mappingParams.save_texture = true;  // Scene texture will be recorded

    cout << "[cameraController::initSVO] Done..." << endl;
	return true;
}


// Initialize SVO params.
bool cameraController::initSVO(char *filePathSVO)
{
    // set svo parameters for the camera
    cout << endl;
    cout << "[cameraController::initSVO] Set SVO Parameters..." << endl;
    //String input_path(filePathSVO);
    initParams.input.setFromSVOFile(filePathSVO);
    initParams.svo_real_time_mode = true;
    cout << "[cameraController::initSVO] Done..." << endl;
    return true;
}


// Initialize Record params.
bool cameraController::initRecord(char *filePathSVO)
{
    // set record parameters for the camera
    cout << endl;
    cout << "[cameraController::initRecord] Set Record Parameters..." << endl;
    String outputPath(filePathSVO);
    recordingParams.video_filename = outputPath;
    recordingParams.compression_mode = SVO_COMPRESSION_MODE::H264;
    cout << "[cameraController::initRecord] Done..." << endl;
    return true;
}


// Open the camera.
bool cameraController::openCamera(void)
{
    cout << endl;
    cout << "[cameraController::openCamera] Opening ZED camera..." << endl;
    cout << endl;
    returnedState = zed.open(initParams);
    cout << "[cameraController::openCamera] checking returned state..." << endl;
    if (returnedState != ERROR_CODE::SUCCESS) 
    {
        cout << "[cameraController::openCamera] Error:  " << returnedState << ", exit program." << endl;
        zed.close(); // close camera.
        exit(-1); // exit program.
    }
    else
    {
        cout << "[cameraController::openCamera] Opened: " << returnedState << endl;
    }
    // allocate MAT pointcloud memory
    slPointcloud.alloc(zed.getCameraInformation().camera_configuration.resolution, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);
    return true;
}


// Close the camera.
bool cameraController::closeCamera(void)
{
    cout << endl;
    cout << "[cameraController::closeCamera] Closing the ZED camera..." << endl;
    slPointcloud.free();
    zed.close(); // close the zed camera
    cout << "[cameraController::closeCamera] SUCCES" << endl;
	return EXIT_SUCCESS;
}


bool cameraController::getSlPointcloud(void)
{
    cout << "[cameraController::getSlPointcloud] in getslpointcloud" << endl;
    returnedState = zed.grab(runtimeParams);

    int counter = 0;
    while(returnedState != sl::ERROR_CODE::SUCCESS)
    {
        returnedState = zed.grab(runtimeParams);
        counter++;

        if(returnedState == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
        {
        cout << "[cameraController::getSlPointcloud] End of the SVO file has been reached!" << endl;
        return false;
        }
        if((counter > 30))
        {
        cout << "[cameraController::getSlPointcloud] Tried to grab zed to many times" << endl;
        return false;
        }
    }

    // get sl pointcloud.
    cout << "[cameraController::getSlPointcloud] grabbing pointcloud" << endl;
    zed.retrieveMeasure(slPointcloud, sl::MEASURE::XYZRGBA);
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     SideFunctions     ///////////////////////////////////////////////////////////////////////////////////

// Enable positional Tracking.
bool cameraController::enableTracking(void)
{
    returnedState = zed.enablePositionalTracking(trackingParams); // enable positionaltracking
    if (returnedState != ERROR_CODE::SUCCESS) // check if failed.
    { 
        cout << "[Camera Tracking] Error " << returnedState << ", exit program." << endl;
        zed.close(); // close camera.
        exit(-1); // exit program.
    }
}


// Disable positional Tracking.
bool cameraController::disableTracking(void)
{
    zed.disablePositionalTracking();
    return true;
}


// Enable Spatial Mapping.
bool cameraController::enableMapping(void)
{
    returnedState = zed.enableSpatialMapping(mappingParams); // enable spatial mapping.
    if (returnedState != ERROR_CODE::SUCCESS) //check if succes.
    {
        cout << "[Camera Mapping] Error " << returnedState << ", exit program." << endl;
        zed.close(); // close camera.
        exit(-1); // exit program.
    }
}


// Disable spatial mapping.
bool cameraController::disableMapping(void)
{
    zed.disableSpatialMapping();
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////