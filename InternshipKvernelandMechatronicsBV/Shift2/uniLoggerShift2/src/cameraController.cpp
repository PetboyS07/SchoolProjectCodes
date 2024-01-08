//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger software
// Filename:    cameraController.cpp
// Changed:     04-01-2023 | 12:20
//
// Discription: This file contains all memberfunctions from the cameraController class.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header Includes
#include "cameraController.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Parameters     //////////////////////////////////////////////////////////////////////////////////////

sl::Camera zed;
sl::Mat slPointcloud;

// for camera configration
sl::InitParameters initParams; // Params to initialize camera.
sl::RuntimeParameters runtimeParams; // Params to initialize video.
sl::PositionalTrackingParameters trackingParams; // Params to initialize positional tracking.
sl::SpatialMappingParameters mappingParams; // Params to initialize spatial mapping
sl::Pose zedPose; // create position from Pose class
auto returnedState = sl::ERROR_CODE::SUCCESS; // to check if data is valid.

// for recording svo
sl::RecordingParameters recordingParams; // Params to initialize recording.
sl::RecordingStatus recStatus; // Status to keep track of the recording.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Functions     ///////////////////////////////////////////////////////////////////////////////////////


// Initialyse the camera. 
bool initCamera(void) 
{
  //cout << endl;
  std::cout << "[cameraController::initCamera] Begin initializing the ZED camera..." << std::endl;

	// set init parameters for the camera.
  std::cout << "[cameraController::initCamera] Set Init Parameters..." << std::endl;
  //initParams.sdk_verbose = false; // Enable verbose logging
  initParams.camera_resolution = sl::RESOLUTION::HD1080;
  initParams.camera_fps = 30;
  initParams.depth_mode = sl::DEPTH_MODE::ULTRA;
  initParams.coordinate_units = sl::UNIT::METER; // might be MILIMETER?
  initParams.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	initParams.enable_image_enhancement = true;
    
	// set runtime parameters for the camera.
  std::cout << "[cameraController::initCamera] Set Runtime Parameters..." << std::endl;
	runtimeParams.sensing_mode = sl::SENSING_MODE::STANDARD;
	runtimeParams.enable_depth = true;
  runtimeParams.confidence_threshold = 100;
  runtimeParams.texture_confidence_threshold = 100;
  runtimeParams.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
  runtimeParams.remove_saturated_areas = true;

  std::cout << "[cameraController::initCamera] Done..." << std::endl;
	return true;
}


// Initialize SVO params.
bool initSVO(char *filePathSVO)
{
    // set svo parameters for the camera
    std::cout << std::endl;
    std::cout << "[cameraController::initSVO] Set SVO Parameters..." << std::endl;
    //String input_path(filePathSVO);
    initParams.input.setFromSVOFile(filePathSVO);
    initParams.svo_real_time_mode = true;
    std::cout << "[cameraController::initSVO] Done..." << std::endl;
    return true;
}


// Initialize Record params.
bool initRecord(char *filePathSVO)
{
    // set record parameters for the camera
    std::cout << std::endl;
    std::cout << "[cameraController::initRecord] Set Record Parameters..." << std::endl;
    sl::String outputPath(filePathSVO);
    recordingParams.video_filename = outputPath;
    recordingParams.compression_mode = sl::SVO_COMPRESSION_MODE::H264;
    std:: cout << "[cameraController::initRecord] Done..." << std::endl;
    return true;
}


// Open the camera.
bool openCamera(void)
{
  std::cout << std::endl;
  std::cout << "[cameraController::openCamera] Opening ZED camera..." << std::endl;
  std::cout << std::endl;
  returnedState = zed.open(initParams);
  std::cout << "[cameraController::openCamera] checking returned state..." << std::endl;
  if (returnedState != sl::ERROR_CODE::SUCCESS) 
  {
      std::cout << "[cameraController::openCamera] Error:  " << returnedState << ", exit program." << std::endl;
      zed.close(); // close camera.
      exit(-1); // exit program.
  }
  else
  {
      std::cout << "[cameraController::openCamera] Opened: " << returnedState << std::endl;
  }
  // allocate MAT pointcloud memory
  slPointcloud.alloc(zed.getCameraInformation().camera_configuration.resolution, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);
  return true;
}


// Close the camera.
void closeCamera(void)
{
  std::cout << std::endl;
  std::cout << "[cameraController::closeCamera] Closing the ZED camera..." << std::endl;
  slPointcloud.free();
  zed.close(); // close the zed camera
  std::cout << "[cameraController::closeCamera] SUCCES" << std::endl;
}


bool getSlPointcloud(void)
{
  std::cout << "[cameraController::getSlPointcloud] in getslpointcloud" << std::endl;
  returnedState = zed.grab(runtimeParams);

  int counter = 0;
  while(returnedState != sl::ERROR_CODE::SUCCESS)
  {
    returnedState = zed.grab(runtimeParams);
    counter++;

    if(returnedState == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
    {
      std::cout << "[cameraController::gestSlPointcloud] SVO file is finisched!" << std::endl;
      return false;
    }
    if((counter > 30))
    {
      std::cout << "[cameraController::gestSlPointcloud] Tried to grab zed to many times." << std::endl;
      return false;
    }
  }
	
	// get pointcloud
  std::cout << "[cameraController::getSlPointcloud] grabbing pointcloud" << std::endl;
  zed.retrieveMeasure(slPointcloud, sl::MEASURE::XYZRGBA);
  return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     SideFunctions     ///////////////////////////////////////////////////////////////////////////////////

// Enable positional Tracking.
bool enableTracking(void)
{
  returnedState = zed.enablePositionalTracking(trackingParams); // enable positionaltracking
  if (returnedState != sl::ERROR_CODE::SUCCESS) // check if failed.
  { 
      std::cout << "[Camera Tracking] Error " << returnedState << ", exit program." << std::endl;
      zed.close(); // close camera.
      exit(-1); // exit program.
  }
}


// Disable positional Tracking.
bool disableTracking(void)
{
  zed.disablePositionalTracking();
  return true;
}


// Enable Spatial Mapping.
bool enableMapping(void)
{
  returnedState = zed.enableSpatialMapping(mappingParams); // enable spatial mapping.
  if (returnedState != sl::ERROR_CODE::SUCCESS) //check if succes.
  {
      std::cout << "[Camera Mapping] Error " << returnedState << ", exit program." << std::endl;
      zed.close(); // close camera.
      exit(-1); // exit program.
  }
}


// Disable spatial mapping.
bool disableMapping(void)
{
  zed.disableSpatialMapping();
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////



