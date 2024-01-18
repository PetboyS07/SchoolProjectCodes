//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        25-09-2023
// Project:     Unilog
// Filename:    functionControl.cpp
// Changed:     03-01-2023 | 14:35
//
// Discription: This file contains all old function from unilogMain.
//              In here u find a serie of functions to relize a application for the unilog project.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header includes
#include "functionController.h"
#include "GLViewer.hpp"
#include "utils.hpp"

// Vision includes
#include <GL/glew.h>
#include <GLFW/glfw3.h> 
#include <cuda_runtime.h>
#include <cuda.h>
#include <cuda_gl_interop.h>
#include <opencv2/opencv.hpp>

// MISC includes
#include <utility>
#include <iostream>
#include <stdio.h>
#include <ctime>
#include <string>
#include <fstream>

// PCL includes
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

// Sample includes
#include <thread>
#include <mutex>

//namespaces
using namespace std;
using namespace sl;

auto returned_state = ERROR_CODE::SUCCESS;
const int MAX_CHAR = 128;

SensorsData sensorData;
SensorsData::TemperatureData temperatureData;
// for transforming svo to pointcloud
Mat data_cloud; // sl format pointcloud
std::thread zed_callback;
mutex mutex_input;
Resolution cloud_res;
bool stop_signal;
bool has_data;


// pointcloud transformation.
shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
inline float convertColor(float colorIn);



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Setup and Init     //////////////////////////////////////////////////////////////////////////////////

inline void setTxt(sl::float3 value, char* ptr_txt) 
{
    snprintf(ptr_txt, MAX_CHAR, "%3.2f x; %3.2f y; %3.2f z", value.x, value.y, value.z);
}


// Initialyse the camera. 
bool functionController::initCamera(void) 
{
    cout << endl;
    cout << "[Camera Init] Begin initializing the ZED camera..." << endl;

      //initParams.sdk_verbose = false; // Enable verbose logging
    initParameters.camera_resolution = sl::RESOLUTION::HD720; // test 1: HD720 | test 2: HD1080
    initParameters.camera_fps = 15; // test: 15 FPS | test 2: 30FPS
    initParameters.depth_mode = sl::DEPTH_MODE::ULTRA; // evt performance als het langzaam is.
    initParameters.coordinate_units = sl::UNIT::METER; // might be MILIMETER? NOPE!
    initParameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	initParameters.enable_image_enhancement = true;
    
	// set runtime parameters for the camera.
	runtimeParameters.sensing_mode = sl::SENSING_MODE::STANDARD;
	runtimeParameters.enable_depth = true;
    runtimeParameters.confidence_threshold = 100;
    runtimeParameters.texture_confidence_threshold = 100;
    runtimeParameters.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
    runtimeParameters.remove_saturated_areas = true;

	// set init parameters for the camera.
    cout << "[Camera Init] Set Init Parameters..." << endl;
    returned_state = ERROR_CODE::SUCCESS;
    // initParameters.sdk_verbose = false; // Enable verbose logging
    // initParameters.camera_resolution = RESOLUTION::HD1080;
    // initParameters.camera_fps = 1;
    // initParameters.depth_mode = DEPTH_MODE::ULTRA;
    // initParameters.coordinate_units = UNIT::METER; // might be MILIMETER?
    // initParameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    
	// // set runtime parameters for the camera.
    // cout << "[Camera Init] Set Runtime Parameters..." << endl;
    // runtimeParameters.confidence_threshold = 40; // 50?
    // runtimeParameters.texture_confidence_threshold = 100;
    // runtimeParameters.enable_depth = true;

    // set positional tracking params
    cout << "[Camera Init] Set Tracking Parameters..." << endl;
    trackingParameters.enable_pose_smoothing = true; // corrects small drifts

    // set spetial mapping params
    cout << "[Camera Init] Set Mapping Parameters..." << endl;
    mappingParameters.resolution_meter = 0.03; // set resolution to 3 cm.
    mappingParameters.range_meter = 3; // set maximum depth maping range to 3 m.
    mappingParameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
    mappingParameters.save_texture = true;  // Scene texture will be recorded

    cout << "[Camera Init] Done..." << endl;
	return true;
}


// Initialize SVO params.
bool functionController::initSVO(char *filePathSVO)
{
    // set svo parameters for the camera
    cout << endl;
    cout << "[Camera Init] Set SVO Parameters..." << endl;
    //String input_path(filePathSVO);
    initParameters.input.setFromSVOFile(filePathSVO);
    initParameters.svo_real_time_mode = true;
    cout << "[Camera Init] Done..." << endl;
    return true;
}


// Initialize Record params.
bool functionController::initRecord(char *filePathSVO)
{
    // set record parameters for the camera
    cout << endl;
    cout << "[Camera Init] Set Record Parameters..." << endl;
    String outputPath(filePathSVO);
    recordingParameters.video_filename = outputPath;
    recordingParameters.compression_mode = SVO_COMPRESSION_MODE::H264;
    cout << "[Camera Init] Done..." << endl;
    return true;
}


// Open the camera.
bool functionController::openCamera(void)
{
    cout << endl;
    cout << "[Camera Opening] Opening ZED camera..." << endl;
    cout << endl;
    returned_state = open(initParameters);
    cout << "[Camera Opening] checking returned state..." << endl;
    if (returned_state != ERROR_CODE::SUCCESS) 
    {
        cout << "[Camera Opening] Error:  " << returned_state << ", exit program." << endl;
        close(); // close camera.
        exit(-1); // exit program.
    }
    else
    {
        cout << "[Camera Opening] Opened: " << returned_state << endl;
    }
    return true;
}


// Close the camera.
bool functionController::closeCamera(void)
{
    cout << endl;
    cout << "[Camera Clossing] Closing the ZED camera..." << endl;
    close(); // close the zed camera
    cout << "[Camera Clossing] SUCCES" << endl;
	return EXIT_SUCCESS;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Main Functions     //////////////////////////////////////////////////////////////////////////////////


// Application to record a SVO.
bool functionController::recordSVO(void)
{
    cout << endl;
    cout << "[Camera Recording] Setting up recording..." << endl;
    auto resolution = getCameraInformation().camera_configuration.resolution; // get camera resolution.


    // Define OpenCV window size (resize to max 720/404).
    Resolution low_resolution(min(720, (int)resolution.width) * 2, min(404, (int)resolution.height));
    Mat live_image(low_resolution, MAT_TYPE::U8_C4, MEM::CPU);
    cv::Mat live_image_ocv = slMat2cvMat(live_image);

    // Setup for Opencv Viewer.
    auto textColor = CV_RGB(255, 255, 255); //white
    auto textBackgroundColor = CV_RGB(0, 0, 0); //black
    auto textFont = cv::FONT_HERSHEY_TRIPLEX;
    auto textScaler = 0.7;
    auto textTickness = 1;
    auto textBackgroundTickness = textTickness + 1;
    auto lineType = cv::LINE_AA;
    auto textPosMid = (low_resolution.width/2)+10;
    
    // Keybinds.
    char key = ' ';
    cout << "[Camera Recording] Keybinds: " << endl;
    cout << "[Camera Recording] Press 'b' to begin SVO recording" << endl;
    cout << "[Camera Recording] Press 's' to stop SVO recording" << endl;
    cout << "[Camera Recording] Press 'q' to exit..." << endl;
    cout << endl;

    // Setup for sensordata
    enableTracking(); // Enable positional Tracking.
    char textTranslation[MAX_CHAR];
    char textOrientation[MAX_CHAR];
    char textRotation[MAX_CHAR];
    char textImuOrientation[MAX_CHAR];
    char textImuAcceleration[MAX_CHAR];
    char textImuAngularVelocity[MAX_CHAR];
    float textImuAltitude = 0.0;
    float textTemperatureImu;
    Timestamp lastImuTs = 0;
    Timestamp lastBarometerTs = 0;
    
    // setup loop.
    int framesRecorded = 0;
    bool startRecord = false;

    // start viewer
    while (key != 'q') // close when "q".
    {
        auto grabErr = grab(); // grab camera.

        if (grabErr <= ERROR_CODE::SUCCESS) // check if camera can be reached.
        {
            // Get the side by side image
            retrieveImage(live_image, VIEW::SIDE_BY_SIDE, MEM::CPU, low_resolution);
            
            // Draw Mid line to split camera feed
            cv::line(live_image_ocv,cv::Point((low_resolution.width/2),0),cv::Point(low_resolution.width/2,low_resolution.height),textBackgroundColor,3); // params: image, starting coordinates, ending coordinates, color, line width, line type: default cv.LINE_8
            cv::line(live_image_ocv,cv::Point((low_resolution.width/2),0),cv::Point(low_resolution.width/2,low_resolution.height),textColor,2); // params: image, starting coordinates, ending coordinates, color, line width, line type: default cv.LINE_8
            
            // Create text input for resolution params
            string dispWindowResolution = "Window Resolution: " + std::to_string(low_resolution.height) + " X " + std::to_string(low_resolution.width);
            string dispSVOResolution =    "SVO Resolution:    " + std::to_string(resolution.height) + " X " + std::to_string(resolution.width);
            cv::putText(live_image_ocv,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textBackgroundColor, textBackgroundTickness, lineType); //target image //text //top-left position //font color
            cv::putText(live_image_ocv,dispSVOResolution,cv::Point(1150, 45),textFont,0.5,textBackgroundColor, textBackgroundTickness, lineType); //target image //text //top-left position //font color
            cv::putText(live_image_ocv,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
            cv::putText(live_image_ocv,dispSVOResolution,cv::Point(1150, 45),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color

            // create text input for keybinds
            cv::putText(live_image_ocv,"Press 'b' to begin SVO recording", cv::Point(10, 25),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(live_image_ocv,"Press 's' to stop SVO recording", cv::Point(10, 50),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(live_image_ocv,"Press 'q' to exit...", cv::Point(10, 75),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(live_image_ocv,"Press 'b' to begin SVO recording", cv::Point(10, 25),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(live_image_ocv,"Press 's' to stop SVO recording", cv::Point(10, 50),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(live_image_ocv,"Press 'q' to exit...", cv::Point(10, 75),textFont,textScaler,textColor,textTickness, lineType);

            // Create text input for position data.   
            getPosition(zedPose, REFERENCE_FRAME::WORLD); // Get the pose of the left eye of the camera with reference to the world frame  
            setTxt(zedPose.getOrientation(), textTranslation); // get orientation and put in in text
            setTxt(zedPose.getTranslation(), textOrientation); // get translation and put in text
            setTxt(zedPose.getEulerAngles(), textRotation); // get angle and put in text
            string dispTranslation = "Translation: " + string(textTranslation);
            string dispOrientation = "Orientation: " + string(textOrientation);
            string dispRotation =    "Rotation:    " + string(textRotation);
            cv::putText(live_image_ocv,"Camera Pose:", cv::Point(10, 300),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(live_image_ocv,dispTranslation, cv::Point(10, 325),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(live_image_ocv,dispOrientation, cv::Point(10, 350),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(live_image_ocv,dispRotation, cv::Point(10, 375),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(live_image_ocv,"Camera Pose:", cv::Point(10, 300),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(live_image_ocv,dispTranslation, cv::Point(10, 325),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(live_image_ocv,dispOrientation, cv::Point(10, 350),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(live_image_ocv,dispRotation, cv::Point(10, 375),textFont,textScaler,textColor,textTickness, lineType);

            // Create text input for IMU data.
            getSensorsData(sensorData, TIME_REFERENCE::IMAGE); // Get IMU data at the time the IMAGE was captured      
            if (sensorData.imu.timestamp > lastImuTs) // Check if IMU data has been updated
            {   setTxt(sensorData.imu.pose.getOrientation(), textImuOrientation); // get orientation from imu and put in in text
                setTxt(sensorData.imu.linear_acceleration, textImuAcceleration); // getacceleration from imu and put in in text
                setTxt(sensorData.imu.angular_velocity, textImuAngularVelocity); // get angular velocity from imu and put in in text
                lastImuTs = sensorData.imu.timestamp;
            }       
            if (sensorData.barometer.timestamp > lastBarometerTs) // Check if IMU data has been updated
            {
                textImuAltitude = sensorData.barometer.relative_altitude;
                lastBarometerTs = sensorData.imu.timestamp;
            }
            temperatureData = sensorData.temperature;
            temperatureData.get(SensorsData::TemperatureData::SENSOR_LOCATION::IMU, textTemperatureImu);
            string dispImuOrientation =     "Orientation:      " + string(textImuOrientation);
            string dispImuAcceleration =    "Acceleration:     " + string(textImuAcceleration);
            string dispImuAngularVelocity = "Angular Velocity: " + string(textImuAngularVelocity);
            string dispImuTemperature =     "Temperature:      " + to_string(textTemperatureImu);
            string dispAltitude = "Altitude:    " + to_string(textImuAltitude);
            cv::putText(live_image_ocv,"IMU Data:", cv::Point(textPosMid, 300),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(live_image_ocv,dispImuOrientation, cv::Point(textPosMid, 325),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(live_image_ocv,dispImuAcceleration, cv::Point(textPosMid, 350),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(live_image_ocv,dispImuAngularVelocity, cv::Point(textPosMid, 375),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(live_image_ocv,dispImuTemperature, cv::Point(textPosMid, 400),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(live_image_ocv,dispAltitude, cv::Point(10, 400),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(live_image_ocv,"IMU Data:", cv::Point(textPosMid, 300),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(live_image_ocv,dispImuOrientation, cv::Point(textPosMid, 325),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(live_image_ocv,dispImuAcceleration, cv::Point(textPosMid, 350),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(live_image_ocv,dispImuAngularVelocity, cv::Point(textPosMid, 375),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(live_image_ocv,dispImuTemperature, cv::Point(textPosMid, 400),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(live_image_ocv,dispAltitude, cv::Point(10, 400),textFont,textScaler,textColor,textTickness, lineType);
            
            // Viewer
            cv::namedWindow("SVO Record Viewer"); // name the window
            cv::moveWindow("SVO Record Viewer", 240,600); // move and fix viewer on the screen
            cv::imshow("SVO Record Viewer", live_image_ocv); // show the viewer
            key = cv::waitKey(10); // wait for a keypres

            if (key == 'b') 
            {
                // enable recording
                cout << "[Camera Recording] Enable Recording..." << endl;
                cout << endl;  
                returned_state = enableRecording(recordingParameters); // start recording svo.
                cout << endl;

                if (returned_state != ERROR_CODE::SUCCESS)  // notify when recording gives error.
                {
                    cout << "[Camera Recording] Enable recording: " << returned_state << endl;
                    close(); //close zed.
                    exit(-1); // exit program.
                }
                else // notify when recording gives succes.
                {
                    cout << "[Camera Recording] Enable recording: " << returned_state << endl;  
                }

                cout << endl;
                startRecord = true; // begin record bool.
                cout << endl;
                cout << "[Camera Recording] SVO is recording..." << endl;
            } 


            while (startRecord) // recording loop.
            {
                // Get the side by side image
                retrieveImage(live_image, VIEW::SIDE_BY_SIDE, MEM::CPU, low_resolution);
                
                // Draw Mid line to split camera feed
                cv::line(live_image_ocv,cv::Point((low_resolution.width/2),0),cv::Point(low_resolution.width/2,low_resolution.height),textBackgroundColor,3); // params: image, starting coordinates, ending coordinates, color, line width, line type: default cv.LINE_8
                cv::line(live_image_ocv,cv::Point((low_resolution.width/2),0),cv::Point(low_resolution.width/2,low_resolution.height),textColor,2); // params: image, starting coordinates, ending coordinates, color, line width, line type: default cv.LINE_8
            
                // Create text input for resolution params
                string dispWindowResolution = "Window Resolution: " + std::to_string(low_resolution.height) + " X " + std::to_string(low_resolution.width);
                string dispSVOResolution =    "SVO Resolution:    " + std::to_string(resolution.height) + " X " + std::to_string(resolution.width);
                cv::putText(live_image_ocv,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textBackgroundColor, textBackgroundTickness, lineType); //target image //text //top-left position //font color
                cv::putText(live_image_ocv,dispSVOResolution,cv::Point(1150, 45),textFont,0.5,textBackgroundColor, textBackgroundTickness, lineType); //target image //text //top-left position //font color
                cv::putText(live_image_ocv,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
                cv::putText(live_image_ocv,dispSVOResolution,cv::Point(1150, 45),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
            
                // create text input for keybinds
                cv::putText(live_image_ocv,"Recording...",cv::Point(10, 25),textFont,textScaler,textBackgroundColor, textBackgroundTickness, lineType); //target image //text //top-left position //font color
                cv::putText(live_image_ocv,"Press 's' to stop SVO recording", cv::Point(10, 50),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(live_image_ocv,"Recording...",cv::Point(10, 25),textFont,textScaler,textColor, textTickness, lineType); //target image //text //top-left position //font color
                cv::putText(live_image_ocv,"Press 's' to stop SVO recording", cv::Point(10, 50),textFont,textScaler,textColor,textTickness, lineType);
                
                // create text input for frames recorded
                string dispFramesRecorded = "Frame # " + std::to_string(framesRecorded);
                cv::putText(live_image_ocv,dispFramesRecorded, cv::Point(textPosMid, 25),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(live_image_ocv,dispFramesRecorded, cv::Point(textPosMid, 25),textFont,textScaler,textColor,textTickness, lineType);
                
                // Create text input for position data.   
                getPosition(zedPose, REFERENCE_FRAME::WORLD); // Get the pose of the left eye of the camera with reference to the world frame  
                setTxt(zedPose.getOrientation(), textTranslation); // get orientation and put in in text
                setTxt(zedPose.getTranslation(), textOrientation); // get translation and put in text
                setTxt(zedPose.getEulerAngles(), textRotation); // get angle and put in text
                string dispTranslation = "Translation: " + string(textTranslation);
                string dispOrientation = "Orientation: " + string(textOrientation);
                string dispRotation =    "Rotation:    " + string(textRotation);
                cv::putText(live_image_ocv,"Camera Pose:", cv::Point(10, 300),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(live_image_ocv,dispTranslation, cv::Point(10, 325),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(live_image_ocv,dispOrientation, cv::Point(10, 350),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(live_image_ocv,dispRotation, cv::Point(10, 375),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(live_image_ocv,"Camera Pose:", cv::Point(10, 300),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(live_image_ocv,dispTranslation, cv::Point(10, 325),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(live_image_ocv,dispOrientation, cv::Point(10, 350),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(live_image_ocv,dispRotation, cv::Point(10, 375),textFont,textScaler,textColor,textTickness, lineType);

                // Create text input for IMU data.
                getSensorsData(sensorData, TIME_REFERENCE::IMAGE); // Get IMU data at the time the IMAGE was captured      
                if (sensorData.imu.timestamp > lastImuTs) // Check if IMU data has been updated
                {   setTxt(sensorData.imu.pose.getOrientation(), textImuOrientation); // get orientation from imu and put in in text
                    setTxt(sensorData.imu.linear_acceleration, textImuAcceleration); // getacceleration from imu and put in in text
                    setTxt(sensorData.imu.angular_velocity, textImuAngularVelocity); // get angular velocity from imu and put in in text
                    lastImuTs = sensorData.imu.timestamp;
                }       
                if (sensorData.barometer.timestamp > lastBarometerTs) // Check if IMU data has been updated
                {
                    textImuAltitude = sensorData.barometer.relative_altitude;
                    lastBarometerTs = sensorData.imu.timestamp;
                }
                temperatureData = sensorData.temperature;
                temperatureData.get(SensorsData::TemperatureData::SENSOR_LOCATION::IMU, textTemperatureImu);
                string dispImuOrientation =     "Orientation:      " + string(textImuOrientation);
                string dispImuAcceleration =    "Acceleration:     " + string(textImuAcceleration);
                string dispImuAngularVelocity = "Angular Velocity: " + string(textImuAngularVelocity);
                string dispImuTemperature =     "Temperature:      " + to_string(textTemperatureImu);
                string dispAltitude = "Altitude:    " + to_string(textImuAltitude);
                cv::putText(live_image_ocv,"IMU Data:", cv::Point(textPosMid, 300),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(live_image_ocv,dispImuOrientation, cv::Point(textPosMid, 325),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(live_image_ocv,dispImuAcceleration, cv::Point(textPosMid, 350),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(live_image_ocv,dispImuAngularVelocity, cv::Point(textPosMid, 375),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(live_image_ocv,dispImuTemperature, cv::Point(textPosMid, 400),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(live_image_ocv,dispAltitude, cv::Point(10, 400),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(live_image_ocv,"IMU Data:", cv::Point(textPosMid, 300),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(live_image_ocv,dispImuOrientation, cv::Point(textPosMid, 325),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(live_image_ocv,dispImuAcceleration, cv::Point(textPosMid, 350),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(live_image_ocv,dispImuAngularVelocity, cv::Point(textPosMid, 375),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(live_image_ocv,dispImuTemperature, cv::Point(textPosMid, 400),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(live_image_ocv,dispAltitude, cv::Point(10, 400),textFont,textScaler,textColor,textTickness, lineType);

                // Display live stream
                cv::imshow("SVO Record Viewer", live_image_ocv);
                key = cv::waitKey(10);

                if (key == 's') // when "s" stop recording.
                {
                    startRecord = false; // stop recording bool.
                    cout << endl;
                    cout << "[Camera Recording] SVO recording stopt." << endl;
                    cout << "[Camera Recording] Press 'q' to exit." << endl;
                    disableRecording(); // disable record.
                    break;
                }
                else if (grab() == ERROR_CODE::SUCCESS) 
                {
                    // Each new frame is added to the SVO file
                    recStatus = getRecordingStatus();

                    if (recStatus.status)
                    {
                        framesRecorded++;
                    }
                            
                    cout << "\r[Camera Recording] || Frame count: " << framesRecorded << " || Press 's' to stop recording." << flush;
                }
                else
                {
                    break;
                }
            }    
        }
        else 
        {
            cout << "Grab ZED: " << grabErr << endl;
            break;
        }
    }
    disablePositionalTracking();
    cv::destroyAllWindows();
    cout << endl;
    return true;
}


// Application to playback a SVO.
bool functionController::playbackSVO(void)
{
    cout << endl;
    cout << "[Camera Playback] Setting up SVO playback..." << endl;
    cout << endl;

    // get camera resolution
    auto resolution = getCameraInformation().camera_configuration.resolution; 

    // Define OpenCV window size (resize to max 720/404)
    Resolution low_resolution(min(720, (int)resolution.width) * 2, min(404, (int)resolution.height));
    Mat svo_image(low_resolution, MAT_TYPE::U8_C4, MEM::CPU);
    cv::Mat svo_image_ocv = slMat2cvMat(svo_image);

    // Setup for Opencv Viewer.
    auto textColor = CV_RGB(255, 255, 255); //white
    auto textBackgroundColor = CV_RGB(0, 0, 0); //black
    auto textFont = cv::FONT_HERSHEY_TRIPLEX;
    auto textScaler = 0.7;
    auto textTickness = 1;
    auto textBackgroundTickness = textTickness + 1;
    auto lineType = cv::LINE_AA;
    auto textPosMid = (low_resolution.width/2)+10;
    
    // Setup for sensordata
    enableTracking(); // Enable positional Tracking.
    char textTranslation[MAX_CHAR];
    char textOrientation[MAX_CHAR];
    char textRotation[MAX_CHAR];
    char textImuOrientation[MAX_CHAR];
    char textImuAcceleration[MAX_CHAR];
    char textImuAngularVelocity[MAX_CHAR];
    float textImuAltitude = 0.0;
    float textTemperatureImu;
    Timestamp lastImuTs = 0;
    Timestamp lastBarometerTs = 0;
    
    // Keybinds
    char key = ' ';
    cout << "[Camera Playback] Keybinds: " << endl;
    cout << "[Camera Playback] Press 's' to save SVO image as a PNG" << endl;
    cout << "[Camera Playback] Press 'f' to jump forward in the video" << endl;
    cout << "[Camera Playback] Press 'b' to jump backward in the video" << endl;
    cout << "[Camera Playback] Press 'q' to exit..." << endl;
    cout << endl;

    // setup svo parameters
    int svo_frame_rate = getInitParameters().camera_fps;
    int nb_frames = getSVONumberOfFrames();
    int svoFrameCounter = 0;
    cout << "[Camera Playback] SVO contains " << nb_frames << " frames" << endl;
    cout << endl;

    while ('q' != key) // Start SVO playback 
    {
        returned_state = grab(runtimeParameters);
        if (returned_state <= ERROR_CODE::SUCCESS) 
        { 
            // Get the side by side image and change variables
            retrieveImage(svo_image, VIEW::SIDE_BY_SIDE, MEM::CPU, low_resolution); 
            int svo_position = getSVOPosition();
            svoFrameCounter++;
            
            // Draw Mid line to split camera feed
            cv::line(svo_image_ocv,cv::Point((low_resolution.width/2),0),cv::Point(low_resolution.width/2,low_resolution.height),textBackgroundColor,3); // params: image, starting coordinates, ending coordinates, color, line width, line type: default cv.LINE_8
            cv::line(svo_image_ocv,cv::Point((low_resolution.width/2),0),cv::Point(low_resolution.width/2,low_resolution.height),textColor,2); // params: image, starting coordinates, ending coordinates, color, line width, line type: default cv.LINE_8
            
            // create text input in viewer for resolution
            string dispWindowResolution = "Window Resolution: " + std::to_string(low_resolution.height) + " X " + std::to_string(low_resolution.width);
            string dispSVOResolution =    "SVO Resolution:    " + std::to_string(resolution.height) + " X " + std::to_string(resolution.width);
            cv::putText(svo_image_ocv,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textBackgroundColor, textBackgroundTickness, lineType); //target image //text //top-left position //font color
            cv::putText(svo_image_ocv,dispSVOResolution,cv::Point(1150, 45),textFont,0.5,textBackgroundColor, textBackgroundTickness, lineType); //target image //text //top-left position //font color  
            cv::putText(svo_image_ocv,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
            cv::putText(svo_image_ocv,dispSVOResolution,cv::Point(1150, 45),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
                
            // create text input in viewer for keybinds
            cv::putText(svo_image_ocv,"Playing SVO...",cv::Point(10, 25),textFont,textScaler,textBackgroundColor, textBackgroundTickness, lineType); //target image //text //top-left position //font color
            cv::putText(svo_image_ocv,"Press 's' to save SVO image as a PNG", cv::Point(10, 50),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,"Press 'f' to jump forward in the video", cv::Point(10, 75),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,"Press 'b' to jump backward in the video", cv::Point(10, 100),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,"Press 'q' to exit", cv::Point(10, 125),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,"Playing SVO...",cv::Point(10, 25),textFont,textScaler,textColor, textTickness, lineType); //target image //text //top-left position //font color
            cv::putText(svo_image_ocv,"Press 's' to save SVO image as a PNG", cv::Point(10, 50),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,"Press 'f' to jump forward in the video", cv::Point(10, 75),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,"Press 'b' to jump backward in the video", cv::Point(10, 100),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,"Press 'q' to exit", cv::Point(10, 125),textFont,textScaler,textColor,textTickness, lineType);
                
            // create text input in viewer for frame counter
            string dispFramesPlayed = "Frame # " + std::to_string(svo_position) +  " / " + std::to_string(nb_frames);
            cv::putText(svo_image_ocv,dispFramesPlayed, cv::Point(textPosMid, 25),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,dispFramesPlayed, cv::Point(textPosMid, 25),textFont,textScaler,textColor,textTickness, lineType);
            
            // Create text input for position data.   
            getPosition(zedPose, REFERENCE_FRAME::WORLD); // Get the pose of the left eye of the camera with reference to the world frame  
            setTxt(zedPose.getOrientation(), textTranslation); // get orientation and put in in text
            setTxt(zedPose.getTranslation(), textOrientation); // get translation and put in text
            setTxt(zedPose.getEulerAngles(), textRotation); // get angle and put in text
            string dispTranslation = "Translation: " + string(textTranslation);
            string dispOrientation = "Orientation: " + string(textOrientation);
            string dispRotation =    "Rotation:    " + string(textRotation);
            cv::putText(svo_image_ocv,"Camera Pose:", cv::Point(10, 300),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,dispTranslation, cv::Point(10, 325),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,dispOrientation, cv::Point(10, 350),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,dispRotation, cv::Point(10, 375),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,"Camera Pose:", cv::Point(10, 300),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,dispTranslation, cv::Point(10, 325),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,dispOrientation, cv::Point(10, 350),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,dispRotation, cv::Point(10, 375),textFont,textScaler,textColor,textTickness, lineType);

            // Create text input for IMU data.
            getSensorsData(sensorData, TIME_REFERENCE::IMAGE); // Get IMU data at the time the IMAGE was captured      
            if (sensorData.imu.timestamp > lastImuTs) // Check if IMU data has been updated
            {   setTxt(sensorData.imu.pose.getOrientation(), textImuOrientation); // get orientation from imu and put in in text
                setTxt(sensorData.imu.linear_acceleration, textImuAcceleration); // getacceleration from imu and put in in text
                setTxt(sensorData.imu.angular_velocity, textImuAngularVelocity); // get angular velocity from imu and put in in text
                lastImuTs = sensorData.imu.timestamp;
            }       
            if (sensorData.barometer.timestamp > lastBarometerTs) // Check if IMU data has been updated
            {
                textImuAltitude = sensorData.barometer.relative_altitude;
                lastBarometerTs = sensorData.imu.timestamp;
            }
            temperatureData = sensorData.temperature;
            temperatureData.get(SensorsData::TemperatureData::SENSOR_LOCATION::IMU, textTemperatureImu);
            string dispImuOrientation =     "Orientation:      " + string(textImuOrientation);
            string dispImuAcceleration =    "Acceleration:     " + string(textImuAcceleration);
            string dispImuAngularVelocity = "Angular Velocity: " + string(textImuAngularVelocity);
            string dispImuTemperature =     "Temperature:      " + to_string(textTemperatureImu);
            string dispAltitude = "Altitude:    " + to_string(textImuAltitude);
            cv::putText(svo_image_ocv,"IMU Data:", cv::Point(textPosMid, 300),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,dispImuOrientation, cv::Point(textPosMid, 325),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,dispImuAcceleration, cv::Point(textPosMid, 350),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,dispImuAngularVelocity, cv::Point(textPosMid, 375),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,dispImuTemperature, cv::Point(textPosMid, 400),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(svo_image_ocv,dispAltitude, cv::Point(10, 400),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType); 
            cv::putText(svo_image_ocv,"IMU Data:", cv::Point(textPosMid, 300),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,dispImuOrientation, cv::Point(textPosMid, 325),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,dispImuAcceleration, cv::Point(textPosMid, 350),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,dispImuAngularVelocity, cv::Point(textPosMid, 375),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,dispImuTemperature, cv::Point(textPosMid, 400),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,dispAltitude, cv::Point(10, 400),textFont,textScaler,textColor,textTickness, lineType); 

            //show the viewer
            cv::namedWindow("SVO Playback Viewer"); // name the window
            cv::moveWindow("SVO Playback Viewer", 240,600); // move and fix viewer on the screen 
            cv::imshow("SVO Playback Viewer", svo_image_ocv); 
            key = cv::waitKey(10); // Wait for keypres
                
            switch (key) 
            {
            case 's': // saves current svo frame
            {
                string name;
                cout << endl;
                cout << "[Camera Playback] file name: ";
                cin >> name;
                svo_image.write(("../../screenshots/capture_" + name + to_string(svo_position) + ".png").c_str());
                cout << "[Camera Playback] Screenshot saved." << endl;
                cout << "[Camera Playback] please wait." << endl;
                break;
            }
            case 'f': // forwards svo frame
            {
                setSVOPosition(svo_position + svo_frame_rate);
                break;
            }
            case 'b': // backwards svo frame
            {
                setSVOPosition(svo_position - svo_frame_rate);
                break;
            }
            }

            // generate ProgressBar
            //ProgressBar((float)(svo_position / (float)nb_frames), 30);
        }
        else if (returned_state == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) // when end of svo file is reached.
        {
            cout << "[Camera Playback] SVO end has been reaced. looping bakc to 0" << endl;
            svoFrameCounter = 0; // reset the svo frame counter
            setSVOPosition(0); // set svo position to the beginning of the file
        }
        else // notify when grab gives error.
        {
            cout << "[Camera Playback] Grab ZED : " << returned_state << endl;
            break;
        }
    }
    disablePositionalTracking();
    cv::destroyAllWindows();
    return true; 
}


// Application to transform SVO to PCL pointcloud.
bool functionController::svoToPCL(void)
{
    // svo path is set in svo init

    cout << "[SVO to PCL] Converting SVO to PCL pointcloud" << endl;

    cloud_res = Resolution(640, 360); // might want to change this to camera resolution
    //cloud_res = Resolution(getCameraInformation().camera_resolution.width, getCameraInformation().camera_resolution.height);
    //cloud_res = getCameraInformation().camera_configuration.resolution;

    // Allocate PCL point cloud at the resolution
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_point_cloud->points.resize(cloud_res.area());

    shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBVisualizer(p_pcl_point_cloud); // Create the PCL point cloud visualizer

    startPclThread(); // Start ZED callback (runs runThread that gets zed pointcloud)

    // Set Viewer initial position
    viewer->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    viewer->setCameraClipDistances(0.1,1000);
    cout << "[SVO to PCL] press 'q' to stop the viewer" << endl;
    while (!viewer->wasStopped()) // Loop until viewer catches the stop signal  original check: !viewer->wasStopped()
    {
        mutex_input.lock(); //Lock to use the point cloud
        float *p_data_cloud = data_cloud.getPtr<float>();
        int index = 0;
        int unvalidMeasurementsCounter = 0;

        

        for (auto &it : p_pcl_point_cloud->points) // Check and adjust points for PCL format
        {
            float X = p_data_cloud[index];
            if (!isValidMeasure(X)) // Checking if it's a valid point
            {
                cout << "\r[SVO to PCL] unvalidMeasurement: " << unvalidMeasurementsCounter << flush;
                it.x = it.y = it.z = it.rgb = 0;
                unvalidMeasurementsCounter++;
            } 
            else 
            {
                it.x = X;
                it.y = p_data_cloud[index + 1];
                it.z = p_data_cloud[index + 2];
                it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format
            }
            index += 4;
        }


        // Unlock data and update Point cloud
        mutex_input.unlock();
        viewer->updatePointCloud(p_pcl_point_cloud);
        viewer->spinOnce(10);

        // save pointcloud
        pclpointcloud = p_pcl_point_cloud;
    }

    viewer->close(); // Close the viewer

    stopPclThread(); // Stop the tread

    cout << endl;
    cout << "[SVO to PCL] Clossing... " << endl;
    return true;
}


// Application to transform SVO to PCL pointcloud.
bool functionController::svoToFilteredPCL(void)
{
    cout << endl;
    cout << "[SVO to Filtered PCL] Converting SVO to a Filtered PCL pointcloud" << endl;

    cloud_res = Resolution(640, 360); // might want to change this to camera resolution
    //cloud_res = Resolution(getCameraInformation().camera_resolution.width, getCameraInformation().camera_resolution.height);
    //cloud_res = getCameraInformation().camera_configuration.resolution;

    // Allocate PCL point cloud at the resolution
    cout << "[SVO to Filtered PCL] Create pointclouds." << endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOriginal(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudOriginal->points.resize(cloud_res.area());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInliers (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOutliers (new pcl::PointCloud<pcl::PointXYZRGB>);
    cout << "[SVO to Filtered PCL] Pointclouds created." << endl;
    
    // Create the filtering object
    cout << "[SVO to Filtered PCL] Setting up Filter.. " << endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloudOriginal); // specify the pointcloud to filter
    cout << "[SVO to Filtered PCL] SOR input cloud set." << endl;
    sor.setMeanK (50); // Number of neighbors to analyse fo each point
    sor.setStddevMulThresh (1.0); // Standard deviation multiplier,
    cout << "[SVO to Filtered PCL] StatisticalOutliersRemoval filter created." << endl;
    
    // Create Viewer
    cout << "[SVO to Filtered PCL] Create viewers." << endl;
    shared_ptr<pcl::visualization::PCLVisualizer> viewerOriginal = createRGBVisualizer(cloudOriginal);
    shared_ptr<pcl::visualization::PCLVisualizer> viewerInliers = createRGBVisualizer(cloudInliers);
    shared_ptr<pcl::visualization::PCLVisualizer> viewerOutliers = createRGBVisualizer(cloudOutliers);
    cout << "[SVO to Filtered PCL] Viewers created." << endl;
    
    
    // Start ZED callback (also starts runThread that gets zed pointcloud)
    cout << "[SVO to Filtered PCL] Starting Thread." << endl;
    //startPclThread(); 

    // Set Viewer initial position
    cout << "[SVO to Filtered PCL] Initialize viewers." << endl;
    viewerOriginal->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    viewerOriginal->setCameraClipDistances(0.1,1000);
    viewerInliers->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    viewerInliers->setCameraClipDistances(0.1,1000);
    viewerOutliers->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    viewerOutliers->setCameraClipDistances(0.1,1000);
    cout << "[PCL Filter Compare] Viewer initialized." << endl;

    cout << "[SVO to Filtered PCL] Starting loop." << endl;
    cout << "[SVO to Filtered PCL] press 'q' to stop the viewer" << endl;
    int nbFrames = getSVONumberOfFrames();
    while (!viewerOriginal->wasStopped() && !viewerInliers->wasStopped() && !viewerOutliers->wasStopped()) // Loop until viewer catches the stop signal  original check: !viewer->wasStopped()
    {
        mutex_input.lock(); //Lock to use the point cloud
        retrieveMeasure(data_cloud, MEASURE::XYZRGBA, MEM::CPU, cloud_res); // retrieve a pointcloud
        int svo_position = getSVOPosition();
        float *p_data_cloud = data_cloud.getPtr<float>();
        int index = 0;
        int unvalidMeasurementsCounter = 0;
        cout << endl;
        cout << "Frame # : " << svo_position << " | " << nbFrames << endl;
        cout << "[SVO to Filtered PCL] Starting SL to PCL transformation." << endl;
        for (auto &it : cloudOriginal->points) // Check and adjust points for PCL format
        {
            float X = p_data_cloud[index];
            if (!isValidMeasure(X)) // Checking if it's a valid point
            {
                cout << "\r[SVO to Filtered PCL] UnvalidMeasurement: " << unvalidMeasurementsCounter << flush;
                it.x = it.y = it.z = it.rgb = 0;
                unvalidMeasurementsCounter++;
            } 
            else 
            {
                it.x = X;
                it.y = p_data_cloud[index + 1];
                it.z = p_data_cloud[index + 2];
                it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format
            }
            index += 4;
        }
        cout << "[SVO to Filtered PCL] SL to PCL Transformed." << endl;

        // Unlock data to acces pointcloud
        mutex_input.unlock();

        cout << "[SVO to Filtered PCL] Starting filter..." << endl;
        sor.setNegative(false);
        sor.filter(*cloudInliers); // all points larger than sd of the mean distance to the query point whill be marked as outliers and removed.
        cout << "[SVO to Filtered PCL] Cloud filtered." << endl;

        // reverse filter to get only the outliers
        cout << "[SVO to Filtered PCL] Reversing filter to negative..." << endl;
        sor.setNegative(true);
        cout << "[SVO to Filtered PCL] Starting Negative filter..." << endl;
        sor.filter(*cloudOutliers);
        cout << "[SVO to Filtered PCL] Outlier cloud created." << endl;

        // Update and open pointcloud viewers
        cout << "[SVO to Filtered PCL] Update and open pointcloud viewers." << endl;
        viewerOriginal->updatePointCloud(cloudOriginal);
        viewerInliers->updatePointCloud(cloudInliers);
        viewerOutliers->updatePointCloud(cloudOutliers);
        viewerOriginal->spinOnce(100);
        viewerInliers->spinOnce(100);
        viewerOutliers->spinOnce(100);

        // usleep(10000); nessesary?
        
        // some code to save the filtered cloud for later?
    }

    // Close the viewer
    cout << "[SVO to Filtered PCL] closing viewers." << endl;
    viewerOriginal->close();
    viewerInliers->close(); 
    viewerOutliers->close(); 
    cout << "[SVO to Filtered PCL] Viewers closed." << endl;

    //stopPclThread(); // Stop the tread
    cout << "[SVO to Filtered PCL] Thread stoped." << endl;
    return true;
}


// This functions start the ZED's thread that grab images and data.
void functionController::startPclThread(void) 
{
    // Start the thread for grabbing ZED data
    stop_signal = false; // set boolian to default: false.
    has_data = false; // set boolian to default: false.
    zed_callback = std::thread(&functionController::runPclThread, this); // start the thread
    while (!has_data) //Wait for data to be grabbed
        sleep_ms(1);
}


//This function loops to get the point cloud from the ZED. It can be considered as a callback.
void functionController::runPclThread() 
{
    while (!stop_signal) 
    {
        if (grab(SENSING_MODE::STANDARD) == ERROR_CODE::SUCCESS) 
        {
        mutex_input.lock(); // lock mutex to prevent from data corruption
        retrieveMeasure(data_cloud, MEASURE::XYZRGBA, MEM::CPU, cloud_res); // retrieve a pointcloud
        mutex_input.unlock(); // unlock mutex again
        has_data = true; // boolian to know if there is new data.
        } else
        sleep_ms(1);
    }
}


// This function frees the ZED, its callback(thread) and the viewer.
void functionController::stopPclThread(void) 
{
    // Stop the thread
    stop_signal = true;
    zed_callback.join();

}


// This function creates a PCL visualizer.
shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) 
{
    // Open 3D viewer and add point cloud
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Pointcloud Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}


// This function convert a RGBA color packed into a packed RGBA PCL compatible format.
inline float convertColor(float colorIn) 
{
    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float *> (&color_uint);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Side Functions     //////////////////////////////////////////////////////////////////////////////////


// Filter for a single pcl pointcloud
bool functionController::pclFilter(char* filepath)
{
    cout << endl;
    cout << "[PCL Filter] Setting up Filter.. " << endl;
    string pointcloudFilePath(filepath);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create pointcloud reader
    pcl::PCDReader reader;
    
    // read pointcloud from given file.
    reader.read<pcl::PointXYZRGB> (pointcloudFilePath, *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud); // specify the pointcloud to filter
    sor.setMeanK (50); // Number of neighbors to analyse fo each point
    sor.setStddevMulThresh (1.0); // Standard deviation multiplier,
    sor.filter (*cloud_filtered); // al point larger than sd of the mean distance to the query point whill be marked as outliers and removed.

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    // write the inliers to disk
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> ("../../pcl/test_inliers.pcd", *cloud_filtered, false);

    // reverse filter to get only the outliers
    sor.setNegative (true);
    sor.filter (*cloud_filtered);

    // write the outliers to disk
    writer.write<pcl::PointXYZRGB> ("../../pcl/test_outliers.pcd", *cloud_filtered, false);

    return true;
}


// viewer for a single pcl pointcloud
bool functionController::pclViewer(char* filepath)
{
    cout << endl;
    cout << "[PCL Viewer] Setting up Viewer.. " << endl;

    // Create pointcloud pointer
    cout << "[Debug] Create pointer" << endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(filepath, *cloud);

    // Create Viewer
    shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBVisualizer(cloud);

    // Set Viewer initial position
    viewer->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    viewer->setCameraClipDistances(0.1,1000);
    viewer->addCoordinateSystem(0.1);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce (100); // show pointcloud
        usleep(10000);
    }

    // Close the viewer
    viewer->close(); 

    cout << endl;
    cout << "[PCL Viewer] Clossing... " << endl;
    return true;
}


// viewer to compaire unfiltered pointcloud with inliers and outliers.
bool functionController::pclFilterCompare(char* filepath)
{
    cout << endl;
    cout << "[PCL Filter Compare] Setting up Filter..." << endl;
    string pointcloudFilePath(filepath);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInliers (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOutliers (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create pointcloud reader
    pcl::PCDReader reader;
    
    // read pointcloud from given file.
    reader.read<pcl::PointXYZRGB> (pointcloudFilePath, *cloud);
    cout << "[PCL Filter Compare] Pointcloud read from filepath." << endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    cout << "[PCL Filter Compare] StatisticalOutliersRemoval filter created." << endl;
    sor.setInputCloud (cloud); // specify the pointcloud to filter
    cout << "[PCL Filter Compare] SOR input cloud set." << endl;
    sor.setMeanK (50); // Number of neighbors to analyse fo each point
    sor.setStddevMulThresh (1.0); // Standard deviation multiplier,
    cout << "[PCL Filter Compare] starting filter..." << endl;
    sor.filter (*cloudInliers); // al point larger than sd of the mean distance to the query point whill be marked as outliers and removed.
    cout << "[PCL Filter Compare] Cloud filterd." << endl;

    // reverse filter to get only the outliers
    sor.setNegative (true);
    cout << "[PCL Filter Compare] starting filter..." << endl;
    sor.filter (*cloudOutliers);
    cout << "[PCL Filter Compare] outlier cloud created." << endl;

    // Create Viewer
    shared_ptr<pcl::visualization::PCLVisualizer> viewerOriginal = createRGBVisualizer(cloud);
    shared_ptr<pcl::visualization::PCLVisualizer> viewerInliers = createRGBVisualizer(cloudInliers);
    shared_ptr<pcl::visualization::PCLVisualizer> viewerOutliers = createRGBVisualizer(cloudOutliers);
    cout << "[PCL Filter Compare] Viewers created." << endl;

    // Set Viewer initial position
    viewerOriginal->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    viewerOriginal->setCameraClipDistances(0.1,1000);
    viewerInliers->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    viewerInliers->setCameraClipDistances(0.1,1000);
    viewerOutliers->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    viewerOutliers->setCameraClipDistances(0.1,1000);
    cout << "[PCL Filter Compare] Viewer settings mate." << endl;

    while (!viewerOriginal->wasStopped() && !viewerInliers->wasStopped() && !viewerOutliers->wasStopped())
    {
        viewerOriginal->spinOnce (100); // show pointcloud
        viewerInliers->spinOnce (100); // show pointcloud
        viewerOutliers->spinOnce (100); // show pointcloud
        usleep(10000);
    }

    // Close the viewer
    viewerOriginal->close();
    viewerInliers->close(); 
    viewerOutliers->close(); 
    cout << "[PCL Filter Compare] Viewers closed" << endl;
    cout << endl;
    cout << "[PCL Viewer Compare] Clossing... " << endl;
    return true;
}


// Enable positional Tracking.
bool functionController::enableTracking(void)
{
    returned_state = enablePositionalTracking(trackingParameters); // enable positionaltracking
    if (returned_state != ERROR_CODE::SUCCESS) // check if failed.
    { 
        cout << "[Camera Tracking] Error " << returned_state << ", exit program." << endl;
        close(); // close camera.
        exit(-1); // exit program.
    }
}


// Start tracking the camera.
bool functionController::trackPosition(int count)
{
    cout << endl;
    cout << "[Camera Tracking] Setting up positional tracking... " << endl;
    enableTracking(); // Enable positional Tracking.
    int counter = 0; // create counter for whileloop
    Pose zedPose; // create position from Pose class
    
    cout << "[Camera Tracking] Start tracking... " << endl;
    // Track the camera position during 500 frames
    while (counter < count) // count default: 500.
    {
        if (grab() == ERROR_CODE::SUCCESS) 
        {

            // Get the pose of the left eye of the camera with reference to the world frame
            getPosition(zedPose, REFERENCE_FRAME::WORLD); 

            // get the translation information
            auto zedTranslation = zedPose.getTranslation();
            // get the orientation information
            auto zedOrientation = zedPose.getOrientation();
            // get the timestamp
            auto ts = zedPose.timestamp.getNanoseconds();

            // Display the translation and timestamp
            cout << "[Camera Tracking] Camera Translation: {" << zedTranslation << "}, Orientation: {" << zedOrientation << "}, timestamp: " << zedPose.timestamp.getNanoseconds() << "ns"  << endl;
            
            // Display IMU data
            if (getSensorsData(sensorData, TIME_REFERENCE::IMAGE) == ERROR_CODE::SUCCESS) // check if imu is availeble
            {
                 // Get IMU data at the time the image was captured
                getSensorsData(sensorData, TIME_REFERENCE::IMAGE);
                //get filtered orientation quaternion
                auto imuOrientation = sensorData.imu.pose.getOrientation();
                // get raw acceleration
                auto acceleration = sensorData.imu.linear_acceleration;

                cout << "[Camera Tracking] IMU Orientation: {" << imuOrientation << "}, Acceleration: {" << acceleration << "}" << endl;
            }
            counter++;
        }
    }
    disablePositionalTracking(); // disables the positional Tracking
    return true;
}


// Enable Spatial Mapping.
bool functionController::enableMapping(void)
{
    returned_state = enableSpatialMapping(mappingParameters); // enable spatial mapping.
    if (returned_state != ERROR_CODE::SUCCESS) //check if succes.
    {
        cout << "[Camera Mapping] Error " << returned_state << ", exit program." << endl;
        close(); // close camera.
        exit(-1); // exit program.
    }
}


// Create Spatial Map.
bool functionController::createSpatialMap(char *filePath, int count)
{
    cout << endl;
    cout << "[Camera Mapping] Setting up spatial mapping... " << endl;
	enableTracking(); // Enable positional tracking.
    enableMapping(); // Enable spatial mapping.
    
    // Grab data during 500 frames.
    int counter = 0;
    sl::Mesh mesh; // Create a mesh object.
    
    cout << "[Camera Mapping] Start mapping... " << endl;
    while (counter < count) // count default: 500.
    {
        // For each new grab, mesh data is updated.
        if (grab() == ERROR_CODE::SUCCESS) 
        {
            // In the background, spatial mapping will use newly retrieved images, depth and pose to update the mesh.
            sl::SPATIAL_MAPPING_STATE mapping_state = getSpatialMappingState();

            // Print spatial mapping state.
            cout << "\r[Camera Mapping] Images captured: " << counter << " / 500  ||  Spatial mapping state: " << mapping_state << "\t" << flush;
            counter++;
        }
    }

    cout << endl;

    // Extract, filter and save the mesh in a obj file.
    cout << "[Camera Mapping] Extracting Mesh..." << endl;
    extractWholeSpatialMap(mesh); // Extract the whole mesh.
    cout << "[Camera Mapping] Filtering Mesh..." << endl;
    mesh.filter(sl::MeshFilterParameters::MESH_FILTER::LOW); // Filter the mesh (remove unnecessary vertices and faces).
    cout << "[Camera Mapping] Apply Textures..." << endl;
    mesh.applyTexture(); // Apply the texture
    cout << "[Camera Mapping] Saving Mesh..." << endl;
    String outputPath(filePath);
    mesh.save(outputPath); // Save the mesh in an obj file.
    cout << "[Camera Mapping] Saved under: " << outputPath << endl;
    
    // Disable tracking and mapping.
    disableSpatialMapping();
    disablePositionalTracking();

    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Get Information Functions     ///////////////////////////////////////////////////////////////////////


// Print Serail Number of the camera.
bool functionController::getSerialNumber(void)
{
    cout << endl;
    cout << "[Camera get] Getting serialnumber..." << endl;
    auto cameraInfos = getCameraInformation(); // Get ZED information
    cout << "[Camera get] ZED Serialnumber: " << cameraInfos.serial_number << endl;
    return true;
}


// Print Image data of the camera.
bool functionController::getImage(int count) 
{
	cout << endl;
    cout << "[Camera get] Getting Image..." << endl;

    int counter = 1; // counter for the whileloop
    Mat image;

	while (counter < count) // get image cout default: 50
    {
        // Grab an image
        if (grab(runtimeParameters) == ERROR_CODE::SUCCESS) // check if there is an issue during capture
        {
            // capture Image
            retrieveImage(image, VIEW::LEFT); // Get the left image
            cout << "\r  [Camera get] Image no. : " << counter+1 << " || Image Resolution: " << image.getWidth() << " x " <<  image.getHeight() << " || Timestamp: " << image.timestamp.data_ns << flush;
            counter++;
        }
	}
    return true;
}
	

// Print depth at the midpoint of the camera image.
bool functionController::getDepth(int count)
{
	cout << endl;
    cout << "[Camera get] Getting Depth..." << endl;
    
    int counter = 0; // counter for the whileloop
    Mat image, depth, pointCloud;
 
    while (counter < count) // get images cout default: 50
    {   
        // Grab an image
        if (grab(runtimeParameters) == ERROR_CODE::SUCCESS) // check if there is an issue during capture
        {
			// capture Image
            retrieveImage(image, VIEW::LEFT); // Get the left image

            // capture depth matrix
            retrieveMeasure(depth, MEASURE::DEPTH);
            
            // capture pointcloud
            retrieveMeasure(pointCloud, MEASURE::XYZRGBA);

            counter++;

            // Get and print distance value in mm at the center of the image
            // We measure the distance camera - object using Euclidean distance
            int x = image.getWidth() / 2;
            int y = image.getHeight() / 2;
            sl::float4 pointCloudValue;
            pointCloud.getValue(x, y, &pointCloudValue);
            float distance = sqrt(pointCloudValue.x*pointCloudValue.x + pointCloudValue.y*pointCloudValue.y + pointCloudValue.z*pointCloudValue.z);
            cout << "Distance to Camera at " << x << ", " << y << ": " << distance << " mm" << endl;
        }
    }
    return true;
}	
	

// Get sensor data from the camera and print it after.	
bool functionController::getSensordata(void)
{
    if(grab() == ERROR_CODE::SUCCESS)
    {
        getSensorsData(sensorData, TIME_REFERENCE::CURRENT); // CURRENT: letimeStamp you retrieve the most recent sensor data available. || IMAGE: letimeStamp you retrieve the closest sensor data to the last image frame.
        printSensorData(); // function that prints the sensordata.
    }
    return true;
}


// Print Orientation, Acceleration, Velocity and Altitude.
bool functionController::printSensorData(void)
{
    cout << endl;
    cout << "[Camera get] Getting sensor data..." << endl;

    if(grab() == ERROR_CODE::SUCCESS)
    {
        getSensorsData(sensorData, TIME_REFERENCE::CURRENT); // CURRENT: letimeStamp you retrieve the most recent sensor data available. || IMAGE: letimeStamp you retrieve the closest sensor data to the last image frame

        Timestamp lastImuTs = 0;
        Timestamp lastBarometerTs = 0;
        int counter = 0;
        
        while (counter < 100)
        {
            // Check if IMU data has been updated
            if (sensorData.imu.timestamp > lastImuTs) 
            {
                cout << endl;
                cout << "[Camera get] IMU Orientation: " << sensorData.imu.pose.getOrientation()  << endl;
                cout << "[Camera get] IMU Linear Acceleration: " << sensorData.imu.linear_acceleration << " [m/sec^2]" << endl;
                cout << "[Camera get] IMU Angular Velocity: " << sensorData.imu.angular_velocity << " [deg/sec]" << endl;
                lastImuTs = sensorData.imu.timestamp;
            }

            // Check if IMU data has been updated
            if (sensorData.barometer.timestamp > lastBarometerTs) 
            {
                cout << "[Camera get] Barometer relative altitude: " << sensorData.barometer.relative_altitude  << endl;
                lastBarometerTs = sensorData.imu.timestamp;
            }
            
            temperatureData = sensorData.temperature;
            float temperatureLeft, temperatureRight, temperatureImu, temperatureBarometer;
            temperatureData.get(SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT, temperatureLeft);
            temperatureData.get(SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_RIGHT, temperatureRight);
            temperatureData.get(SensorsData::TemperatureData::SENSOR_LOCATION::IMU, temperatureImu);
            cout << "Temperature onboard left: " << temperatureLeft << endl;
            cout << "Temperature onboard right: " << temperatureRight << endl;
            cout << "Temperature IMU: " << temperatureImu << endl;

            counter++;
        }
    }
    return true;
}





// Test viewer
bool functionController::testWindow(void)
{
    cout << endl;
    cout << "[Test Window] In test Window" << endl;

    // Enable positional Tracking.
    enableTracking(); 
    
    // create position from Pose class
    Pose zedPose; 

    // get camera resolution.
    auto resolution = getCameraInformation().camera_configuration.resolution; 

    // Define OpenCV window size (resize to max 720/404).
    Resolution low_resolution(min(720, (int)resolution.width) * 2, min(404, (int)resolution.height));
    Mat testWindow(low_resolution, MAT_TYPE::U8_C4, MEM::CPU);
    cv::Mat testWindow_ocv = slMat2cvMat(testWindow);
    cv::Mat extraWindow(404,1440, CV_8UC3, CV_RGB(50, 50, 50)); //green tint
    
    // Define Viewers and what to view
    cv::Mat cameraWindow = testWindow_ocv;
    cv::Mat sensorWindow = extraWindow;

    // Keybinds.
    char key = ' ';
    
    // Setup for Opencv Viewer.
    auto textColor = CV_RGB(255, 255, 255); //white
    auto textBackgroundColor = CV_RGB(0, 0, 0); //black
    auto textFont = cv::FONT_HERSHEY_TRIPLEX;
    auto textScaler = 0.7;
    auto textTickness = 1;
    auto textBackgroundTickness = textTickness + 1;
    auto lineType = cv::LINE_AA;
    auto textPosMid = (low_resolution.width/2)+10;

    // Create text for GUI
    char textTranslation[MAX_CHAR];
    char textOrientation[MAX_CHAR];
    char textRotation[MAX_CHAR];
    char textImuOrientation[MAX_CHAR];
    char textImuAcceleration[MAX_CHAR];
    char textImuAngularVelocity[MAX_CHAR];
    float textImuAltitude = 0.0;
    float textTemperatureImu;
    Timestamp lastImuTs = 0;
    Timestamp lastBarometerTs = 0;

    // setup loop.
    int frames = 0;
    bool begin = false;

    // start viewer
    while (key != 'q') // close when "q".
    {
        auto grabErr = grab(); // grab camera.

        if (grabErr <= ERROR_CODE::SUCCESS) // check if camera can be reached.
        {
            // Get the side by side image
            retrieveImage(testWindow, VIEW::SIDE_BY_SIDE, MEM::CPU, low_resolution);
            sensorWindow = extraWindow;

            // Draw Mid line to split camera feed
            cv::line(cameraWindow,cv::Point((low_resolution.width/2),0),cv::Point(low_resolution.width/2,low_resolution.height),textBackgroundColor,3); // params: image, starting coordinates, ending coordinates, color, line width, line type: default cv.LINE_8
            cv::line(cameraWindow,cv::Point((low_resolution.width/2),0),cv::Point(low_resolution.width/2,low_resolution.height),textColor,2); // params: image, starting coordinates, ending coordinates, color, line width, line type: default cv.LINE_8
            
            // Create text input for resolution params
            string dispWindowResolution = "Window Resolution: " + std::to_string(low_resolution.height) + " X " + std::to_string(low_resolution.width);
            string dispCameraResolution = "SVO Resolution:    " + std::to_string(resolution.height) + " X " + std::to_string(resolution.width);
            cv::putText(cameraWindow,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textBackgroundColor, textBackgroundTickness, lineType); //target image //text //top-left position //font color
            cv::putText(cameraWindow,dispCameraResolution,cv::Point(1150, 45),textFont,0.5,textBackgroundColor, textBackgroundTickness, lineType); //target image //text //top-left position //font color
            cv::putText(cameraWindow,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
            cv::putText(cameraWindow,dispCameraResolution,cv::Point(1150, 45),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
            
            // create text input for keybinds
            cv::putText(cameraWindow,"Press 'b' to begin SVO recording", cv::Point(10, 25),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(cameraWindow,"Press 's' to stop SVO recording", cv::Point(10, 50),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(cameraWindow,"Press 'q' to exit...", cv::Point(10, 75),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(cameraWindow,"Press 'b' to begin SVO recording", cv::Point(10, 25),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(cameraWindow,"Press 's' to stop SVO recording", cv::Point(10, 50),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(cameraWindow,"Press 'q' to exit...", cv::Point(10, 75),textFont,textScaler,textColor,textTickness, lineType);

            // Create text input for position data.   
            getPosition(zedPose, REFERENCE_FRAME::WORLD); // Get the pose of the left eye of the camera with reference to the world frame  
            setTxt(zedPose.getOrientation(), textTranslation); // get orientation and put in in text
            setTxt(zedPose.getTranslation(), textOrientation); // get translation and put in text
            setTxt(zedPose.getEulerAngles(), textRotation); // get angle and put in text
            string dispTranslation = "Translation: " + string(textTranslation);
            string dispOrientation = "Orientation: " + string(textOrientation);
            string dispRotation =    "Rotation:    " + string(textRotation);
            cv::putText(sensorWindow,"Camera Pose:", cv::Point(10, 300),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(sensorWindow,dispTranslation, cv::Point(10, 325),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(sensorWindow,dispOrientation, cv::Point(10, 350),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(sensorWindow,dispRotation, cv::Point(10, 375),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(sensorWindow,"Camera Pose:", cv::Point(10, 300),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(sensorWindow,dispTranslation, cv::Point(10, 325),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(sensorWindow,dispOrientation, cv::Point(10, 350),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(sensorWindow,dispRotation, cv::Point(10, 375),textFont,textScaler,textColor,textTickness, lineType);

            // Create text input for IMU data.
            getSensorsData(sensorData, TIME_REFERENCE::IMAGE); // Get IMU data at the time the IMAGE was captured      
            if (sensorData.imu.timestamp > lastImuTs) // Check if IMU data has been updated
            {   setTxt(sensorData.imu.pose.getOrientation(), textImuOrientation); // get orientation from imu and put in in text
                setTxt(sensorData.imu.linear_acceleration, textImuAcceleration); // getacceleration from imu and put in in text
                setTxt(sensorData.imu.angular_velocity, textImuAngularVelocity); // get angular velocity from imu and put in in text
                lastImuTs = sensorData.imu.timestamp;
            }       
            if (sensorData.barometer.timestamp > lastBarometerTs) // Check if IMU data has been updated
            {
                textImuAltitude = sensorData.barometer.relative_altitude;
                lastBarometerTs = sensorData.imu.timestamp;
            }
            temperatureData = sensorData.temperature;
            temperatureData.get(SensorsData::TemperatureData::SENSOR_LOCATION::IMU, textTemperatureImu);
            string dispImuOrientation =     "Orientation:      " + string(textImuOrientation);
            string dispImuAcceleration =    "Acceleration:     " + string(textImuAcceleration);
            string dispImuAngularVelocity = "Angular Velocity: " + string(textImuAngularVelocity);
            string dispImuTemperature =     "Temperature:      " + to_string(textTemperatureImu);
            string dispAltitude = "Altitude:    " + to_string(textImuAltitude);
            cv::putText(sensorWindow,"IMU Data:", cv::Point(textPosMid, 300),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(sensorWindow,dispImuOrientation, cv::Point(textPosMid, 325),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(sensorWindow,dispImuAcceleration, cv::Point(textPosMid, 350),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(sensorWindow,dispImuAngularVelocity, cv::Point(textPosMid, 375),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(sensorWindow,dispImuTemperature, cv::Point(textPosMid, 400),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(sensorWindow,dispAltitude, cv::Point(10, 400),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
            cv::putText(sensorWindow,"IMU Data:", cv::Point(textPosMid, 300),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(sensorWindow,dispImuOrientation, cv::Point(textPosMid, 325),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(sensorWindow,dispImuAcceleration, cv::Point(textPosMid, 350),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(sensorWindow,dispImuAngularVelocity, cv::Point(textPosMid, 375),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(sensorWindow,dispImuTemperature, cv::Point(textPosMid, 400),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(sensorWindow,dispAltitude, cv::Point(10, 400),textFont,textScaler,textColor,textTickness, lineType);

            // Extra viewer
            cv::namedWindow("Sensor Window");
            cv::moveWindow("Sensor Window", 240,150); // move and fix viewer on the screen
            cv::imshow("Sensor Window", sensorWindow); // show the viewer

            // Test Viewer
            cv::namedWindow("Camera Window"); // name the window
            cv::moveWindow("Camera Window", 240,600); // move and fix viewer on the screen
            cv::imshow("Camera Window", cameraWindow); // show the viewer
            key = cv::waitKey(10); // wait for a keypres

            if (key == 'b') 
            {
                // enable recording
                cout << "[TestDummy] starting..." << endl;
                cout << endl;  

                begin = true; // begin record bool.
                cout << endl;
                cout << "[TestDummy] begon" << endl;
            } 


            while (begin) // begin loop.
            {
                // Get the side by side image
                retrieveImage(testWindow, VIEW::SIDE_BY_SIDE, MEM::CPU, low_resolution);
                sensorWindow = extraWindow;

                // Draw Mid line
                cv::line(cameraWindow,cv::Point((low_resolution.width/2),0),cv::Point(low_resolution.width/2,low_resolution.height),textBackgroundColor,3); // params: image, starting coordinates, ending coordinates, color, line width, line type: default cv.LINE_8
                cv::line(cameraWindow,cv::Point((low_resolution.width/2),0),cv::Point(low_resolution.width/2,low_resolution.height),textColor,2); // params: image, starting coordinates, ending coordinates, color, line width, line type: default cv.LINE_8
            
                // Create text input for resolution params
                string dispWindowResolution = "Window Resolution: " + std::to_string(low_resolution.height) + " X " + std::to_string(low_resolution.width);
                string dispSVOResolution =    "SVO Resolution:    " + std::to_string(resolution.height) + " X " + std::to_string(resolution.width);
                cv::putText(cameraWindow,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textBackgroundColor, textBackgroundTickness, lineType); //target image //text //top-left position //font color
                cv::putText(cameraWindow,dispSVOResolution,cv::Point(1150, 45),textFont,0.5,textBackgroundColor, textBackgroundTickness, lineType); //target image //text //top-left position //font color
                cv::putText(cameraWindow,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
                cv::putText(cameraWindow,dispSVOResolution,cv::Point(1150, 45),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
            
                // create text input for keybinds
                cv::putText(cameraWindow,"begon...",cv::Point(10, 25),textFont,textScaler,textBackgroundColor, textBackgroundTickness, lineType); //target image //text //top-left position //font color
                cv::putText(cameraWindow,"Press 's' to stop", cv::Point(10, 50),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(cameraWindow,"begon...",cv::Point(10, 25),textFont,textScaler,textColor, textTickness, lineType); //target image //text //top-left position //font color
                cv::putText(cameraWindow,"Press 's' to stop", cv::Point(10, 50),textFont,textScaler,textColor,textTickness, lineType);
                
                // create text input for frames recorded
                string dispFrames = "Frame # " + std::to_string(frames);
                cv::putText(cameraWindow,dispFrames, cv::Point(textPosMid, 20),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(cameraWindow,dispFrames, cv::Point(textPosMid, 20),textFont,textScaler,textColor,textTickness, lineType);

                // Create text input for position data.   
                getPosition(zedPose, REFERENCE_FRAME::WORLD); // Get the pose of the left eye of the camera with reference to the world frame  
                setTxt(zedPose.getOrientation(), textTranslation); // get orientation and put in in text
                setTxt(zedPose.getTranslation(), textOrientation); // get translation and put in text
                setTxt(zedPose.getEulerAngles(), textRotation); // get angle and put in text
                string dispTranslation = "Translation: " + string(textTranslation);
                string dispOrientation = "Orientation: " + string(textOrientation);
                string dispRotation =    "Rotation:    " + string(textRotation);
                cv::putText(sensorWindow,"Camera Pose:", cv::Point(10, 300),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(sensorWindow,dispTranslation, cv::Point(10, 325),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(sensorWindow,dispOrientation, cv::Point(10, 350),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(sensorWindow,dispRotation, cv::Point(10, 375),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(sensorWindow,"Camera Pose:", cv::Point(10, 300),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(sensorWindow,dispTranslation, cv::Point(10, 325),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(sensorWindow,dispOrientation, cv::Point(10, 350),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(sensorWindow,dispRotation, cv::Point(10, 375),textFont,textScaler,textColor,textTickness, lineType);

                // Create text input for IMU data.
                getSensorsData(sensorData, TIME_REFERENCE::IMAGE); // Get IMU data at the time the IMAGE was captured      
                if (sensorData.imu.timestamp > lastImuTs) // Check if IMU data has been updated
                {   setTxt(sensorData.imu.pose.getOrientation(), textImuOrientation); // get orientation from imu and put in in text
                    setTxt(sensorData.imu.linear_acceleration, textImuAcceleration); // getacceleration from imu and put in in text
                    setTxt(sensorData.imu.angular_velocity, textImuAngularVelocity); // get angular velocity from imu and put in in text
                    lastImuTs = sensorData.imu.timestamp;
                }       
                if (sensorData.barometer.timestamp > lastBarometerTs) // Check if IMU data has been updated
                {
                    textImuAltitude = sensorData.barometer.relative_altitude;
                    lastBarometerTs = sensorData.imu.timestamp;
                }
                temperatureData = sensorData.temperature;
                temperatureData.get(SensorsData::TemperatureData::SENSOR_LOCATION::IMU, textTemperatureImu);
                string dispImuOrientation =     "Orientation:      " + string(textImuOrientation);
                string dispImuAcceleration =    "Acceleration:     " + string(textImuAcceleration);
                string dispImuAngularVelocity = "Angular Velocity: " + string(textImuAngularVelocity);
                string dispImuTemperature =     "Temperature:      " + to_string(textTemperatureImu);
                string dispAltitude = "Altitude:    " + to_string(textImuAltitude);
                cv::putText(sensorWindow,"IMU Data:", cv::Point(textPosMid, 300),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(sensorWindow,dispImuOrientation, cv::Point(textPosMid, 325),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(sensorWindow,dispImuAcceleration, cv::Point(textPosMid, 350),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(sensorWindow,dispImuAngularVelocity, cv::Point(textPosMid, 375),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(sensorWindow,dispImuTemperature, cv::Point(textPosMid, 400),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(sensorWindow,dispAltitude, cv::Point(10, 400),textFont,textScaler,textBackgroundColor,textBackgroundTickness, lineType);
                cv::putText(sensorWindow,"IMU Data:", cv::Point(textPosMid, 300),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(sensorWindow,dispImuOrientation, cv::Point(textPosMid, 325),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(sensorWindow,dispImuAcceleration, cv::Point(textPosMid, 350),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(sensorWindow,dispImuAngularVelocity, cv::Point(textPosMid, 375),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(sensorWindow,dispImuTemperature, cv::Point(textPosMid, 400),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(sensorWindow,dispAltitude, cv::Point(10, 400),textFont,textScaler,textColor,textTickness, lineType);
                
                // Display live stream
                cv::imshow("Camera Window", cameraWindow);
                cv::imshow("Sensor Window", sensorWindow);
                key = cv::waitKey(10);

                if (key == 's') // when "s" stop recording.
                {
                    begin = false; // stop recording bool.
                    cout << endl;
                    cout << "[Camera Recording] SVO recording stopt." << endl;
                    cout << "[Camera Recording] Press 'q' to exit." << endl;
                    disableRecording(); // disable record.
                    break;
                }
                else if (grab() == ERROR_CODE::SUCCESS) 
                {
                    frames++;
                }
                else
                {
                    break;
                }
            }    
        }
        else 
        {
            cout << "Grab ZED: " << grabErr << endl;
            break;
        }
    }
    disablePositionalTracking();
    cv::destroyAllWindows();

    cout << endl;
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End :)     //////////////////////////////////////////////////////////////////////////////////////
