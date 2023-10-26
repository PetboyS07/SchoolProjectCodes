//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        25-09-2023
// Project:     Unilog
// Filename:    cameraControl.cpp
// Discription: This is a sourche file for the cameraControl class.
//              In here u find a serie of functions for to relize a application for the Unilog project.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header includes
#include "cameraControl.h"
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

// Help function that shows the commands of this program.
void cameraControl::help(void) 
{
    cout << endl;
    cout << "[Usage] In function help." << endl;
    cout << endl;
    cout << "[Usage] Command format: ./filename  --function  svoPath" << endl;
    cout << endl;
    cout << "[Usage] Main Functions: " << endl;
    cout << "[Usage] Type '-r' or '--record' to record svo. (svoOutputPath should be given)" << endl;
    cout << "[Usage] Type '-p' or '--play' to playback svo. (svoInputPath should be given)" << endl;
    cout << "[Usage] Type '-stp' or '--svoToPcl to transform svo to pcl pointcloud." << endl;
    cout << endl;
    cout << "[Usage] Side Functions: " << endl;
    cout << "[Usage] Type '-h' or '--help' for a list of commands." << endl;
    cout << "[Usage] Type '-tp' or '--trackPosition to track ZED camera." << endl;    
    cout << "[Usage] Type '-sm' or '--spatialMapping to start spatial mapping." << endl;    
    cout << endl;
    cout << "[Usage] Get Information Functions: " << endl;
    cout << "[Usage] Type '-gs' or '--getSerialnumber' to get zed serialnumber." << endl;
    cout << "[Usage] Type '-gi' or '--getImage' to get zed image." << endl;
    cout << "[Usage] Type '-gd' or '--getDepth' to get zed depth." << endl;
    cout << "[Usage] Type '-gsd' or '--getSensordata to get sensordata." << endl;
    cout << endl;
    exit(0); // exit the program.
}


// Initialyse the camera. 
bool cameraControl::initCamera(void) 
{
    cout << endl;
    cout << "[Camera Init] Begin initializing the ZED camera..." << endl;

	// set init parameters for the camera.
    cout << "[Camera Init] Set Init Parameters..." << endl;
    returned_state = ERROR_CODE::SUCCESS;
    initParams.sdk_verbose = false; // Enable verbose logging
    initParams.camera_resolution = RESOLUTION::HD1080;
    initParams.camera_fps = 1;
    initParams.depth_mode = DEPTH_MODE::ULTRA;
    initParams.coordinate_units = UNIT::METER; // might be MILIMETER?
    initParams.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    
	// set runtime parameters for the camera.
    cout << "[Camera Init] Set Runtime Parameters..." << endl;
    runtimeParams.confidence_threshold = 40; // 50?
    runtimeParams.texture_confidence_threshold = 100;
    runtimeParams.enable_depth = true;

    // set positional tracking params
    cout << "[Camera Init] Set Tracking Parameters..." << endl;
    trackingParams.enable_pose_smoothing = true; // corrects small drifts

    // set spetial mapping params
    cout << "[Camera Init] Set Mapping Parameters..." << endl;
    mappingParams.resolution_meter = 0.03; // set resolution to 3 cm.
    mappingParams.range_meter = 3; // set maximum depth maping range to 3 m.
    mappingParams.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
    mappingParams.save_texture = true;  // Scene texture will be recorded

    cout << "[Camera Init] Done..." << endl;
	return true;
}


// Initialize SVO params.
bool cameraControl::initSVO(char *filePathSVO)
{
    // set svo parameters for the camera
    cout << endl;
    cout << "[Camera Init] Set SVO Parameters..." << endl;
    //String input_path(filePathSVO);
    initParams.input.setFromSVOFile(filePathSVO);
    initParams.svo_real_time_mode = true;
    cout << "[Camera Init] Done..." << endl;
    return true;
}


// Initialize Record params.
bool cameraControl::initRecord(char *filePathSVO)
{
    // set record parameters for the camera
    cout << endl;
    cout << "[Camera Init] Set Record Parameters..." << endl;
    String outputPath(filePathSVO);
    recordingParams.video_filename = outputPath;
    recordingParams.compression_mode = SVO_COMPRESSION_MODE::H264;
    cout << "[Camera Init] Done..." << endl;
    return true;
}


// Open the camera.
bool cameraControl::openCamera(void)
{
    cout << endl;
    cout << "[Camera Opening] Opening ZED camera..." << endl;
    cout << endl;
    returned_state = open(initParams);
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
bool cameraControl::closeCamera(void)
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
bool cameraControl::recordSVO(void)
{
    cout << endl;
    cout << "[Camera Recording] Setting up recording..." << endl;
    auto resolution = getCameraInformation().camera_configuration.resolution; // get camera resolution.


    // Define OpenCV window size (resize to max 720/404).
    Resolution low_resolution(min(720, (int)resolution.width) * 2, min(404, (int)resolution.height));
    Mat live_image(low_resolution, MAT_TYPE::U8_C4, MEM::CPU);
    cv::Mat live_image_ocv = slMat2cvMat(live_image);

    // Setup for Opencv Viewer.
    auto textColor = CV_RGB(255, 255, 255); //wit
    auto textFont = cv::FONT_HERSHEY_TRIPLEX;
    auto textScaler = 0.7;
    auto textTickness = 1;
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
            
            // Create text input for resolution params
            string dispWindowResolution = "Window Resolution: " + std::to_string(low_resolution.height) + " X " + std::to_string(low_resolution.width);
            string dispSVOResolution = "SVO Resolution: " + std::to_string(resolution.height) + " X " + std::to_string(resolution.width);
            cv::putText(live_image_ocv,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
            cv::putText(live_image_ocv,dispSVOResolution,cv::Point(1150, 45),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color

            // create text input for keybinds
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
                returned_state = enableRecording(recordingParams); // start recording svo.
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
                
                // Create text input for resolution params
                string dispWindowResolution = "Window Resolution: " + std::to_string(low_resolution.height) + " X " + std::to_string(low_resolution.width);
                string dispSVOResolution = "SVO Resolution: " + std::to_string(resolution.height) + " X " + std::to_string(resolution.width);
                cv::putText(live_image_ocv,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
                cv::putText(live_image_ocv,dispSVOResolution,cv::Point(1150, 45),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
            
                // create text input for keybinds
                cv::putText(live_image_ocv,"Recording...",cv::Point(10, 25),textFont,textScaler,textColor, textTickness, lineType); //target image //text //top-left position //font color
                cv::putText(live_image_ocv,"Press 's' to stop SVO recording", cv::Point(10, 50),textFont,textScaler,textColor,textTickness, lineType);
                
                // create text input for frames recorded
                string dispFramesRecorded = "Frame # " + std::to_string(framesRecorded);
                cv::putText(live_image_ocv,dispFramesRecorded, cv::Point(textPosMid, 25),textFont,textScaler,textColor,textTickness, lineType);
                
                // Create text input for position data.   
                getPosition(zedPose, REFERENCE_FRAME::WORLD); // Get the pose of the left eye of the camera with reference to the world frame  
                setTxt(zedPose.getOrientation(), textTranslation); // get orientation and put in in text
                setTxt(zedPose.getTranslation(), textOrientation); // get translation and put in text
                setTxt(zedPose.getEulerAngles(), textRotation); // get angle and put in text
                string dispTranslation = "Translation: " + string(textTranslation);
                string dispOrientation = "Orientation: " + string(textOrientation);
                string dispRotation =    "Rotation:    " + string(textRotation);
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
    cout << endl;
    return true;
}


// Application to playback a SVO.
bool cameraControl::playbackSVO(void)
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

    // Setup opencv
    auto textColor = CV_RGB(255, 255, 255); //wit
    auto textFont = cv::FONT_HERSHEY_TRIPLEX;
    auto textScaler = 0.7;
    auto textTickness = 1;
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
        returned_state = grab(runtimeParams);
        if (returned_state <= ERROR_CODE::SUCCESS) 
        { 
            // Get the side by side image and change variables
            retrieveImage(svo_image, VIEW::SIDE_BY_SIDE, MEM::CPU, low_resolution); 
            int svo_position = getSVOPosition();
            svoFrameCounter++;

            // create text input in viewer for resolution
            string dispWindowResolution = "Window Resolution: " + std::to_string(low_resolution.height) + " X " + std::to_string(low_resolution.width);
            string dispSVOResolution = "SVO Resolution: " + std::to_string(resolution.height) + " X " + std::to_string(resolution.width);
            cv::putText(svo_image_ocv,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
            cv::putText(svo_image_ocv,dispSVOResolution,cv::Point(1150, 45),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
                
            // create text input in viewer for keybinds
            cv::putText(svo_image_ocv,"Playing SVO...",cv::Point(10, 25),textFont,textScaler,textColor, textTickness, lineType); //target image //text //top-left position //font color
            cv::putText(svo_image_ocv,"Press 's' to save SVO image as a PNG", cv::Point(10, 50),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,"Press 'f' to jump forward in the video", cv::Point(10, 75),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,"Press 'b' to jump backward in the video", cv::Point(10, 100),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(svo_image_ocv,"Press 'q' to exit", cv::Point(10, 125),textFont,textScaler,textColor,textTickness, lineType);
                
            // create text input in viewer for frame counter
            string dispFramesPlayed = "Frame # " + std::to_string(svo_position) +  " / " + std::to_string(nb_frames);
            cv::putText(svo_image_ocv,dispFramesPlayed, cv::Point(textPosMid, 25),textFont,textScaler,textColor,textTickness, lineType);
            
            // Create text input for position data.   
            getPosition(zedPose, REFERENCE_FRAME::WORLD); // Get the pose of the left eye of the camera with reference to the world frame  
            setTxt(zedPose.getOrientation(), textTranslation); // get orientation and put in in text
            setTxt(zedPose.getTranslation(), textOrientation); // get translation and put in text
            setTxt(zedPose.getEulerAngles(), textRotation); // get angle and put in text
            string dispTranslation = "Translation: " + string(textTranslation);
            string dispOrientation = "Orientation: " + string(textOrientation);
            string dispRotation =    "Rotation:    " + string(textRotation);
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
                svo_image.write(("[Camera Playback] capture_" + to_string(svo_position) + ".png").c_str());
                break;
            case 'f': // forwards svo frame
                setSVOPosition(svo_position + svo_frame_rate);
                break;
            case 'b': // backwards svo frame
                setSVOPosition(svo_position - svo_frame_rate);
                break;
            }

            // generate ProgressBar
            ProgressBar((float)(svo_position / (float)nb_frames), 30);
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
    return true; 
}


// Application to transform SVO to PCL pointcloud.
bool cameraControl::svoToPCL(void)
{
    cout << "[SVO to PCL] Converting SVO to PCL pointcloud" << endl;

    cloud_res = Resolution(640, 360); // might want to change this to camera resolution
    //cloud_res = Resolution(getCameraInformation().camera_resolution.width, getCameraInformation().camera_resolution.height);
    //cloud_res = getCameraInformation().camera_configuration.resolution;

    // Allocate PCL point cloud at the resolution
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_point_cloud->points.resize(cloud_res.area());

    shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBVisualizer(p_pcl_point_cloud); // Create the PCL point cloud visualizer

    startThread(); // Start ZED callback

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
        pclPointcloud = p_pcl_point_cloud;
    }

    viewer->close(); // Close the viewer

    stopThread(); // Stop the tread

    cout << endl;
    cout << "[SVO to PCL] Clossing... " << endl;
    return true;
}


// This functions start the ZED's thread that grab images and data.
void cameraControl::startThread(void) 
{
    // Start the thread for grabbing ZED data
    stop_signal = false; // set boolian to default: false.
    has_data = false; // set boolian to default: false.
    zed_callback = std::thread(&cameraControl::runThread, this); // start the thread
    while (!has_data) //Wait for data to be grabbed
        sleep_ms(1);
}


//This function loops to get the point cloud from the ZED. It can be considered as a callback.
void cameraControl::runThread() 
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
void cameraControl::stopThread(void) 
{
    // Stop the thread
    stop_signal = true;
    zed_callback.join();

}


// This function creates a PCL visualizer.
shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) 
{
    // Open 3D viewer and add point cloud
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
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

// Enable positional Tracking.
bool cameraControl::enableTracking(void)
{
    returned_state = enablePositionalTracking(trackingParams); // enable positionaltracking
    if (returned_state != ERROR_CODE::SUCCESS) // check if failed.
    { 
        cout << "[Camera Tracking] Error " << returned_state << ", exit program." << endl;
        close(); // close camera.
        exit(-1); // exit program.
    }
}


// Start tracking the camera.
bool cameraControl::trackPosition(int count)
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
bool cameraControl::enableMapping(void)
{
    returned_state = enableSpatialMapping(mappingParams); // enable spatial mapping.
    if (returned_state != ERROR_CODE::SUCCESS) //check if succes.
    {
        cout << "[Camera Mapping] Error " << returned_state << ", exit program." << endl;
        close(); // close camera.
        exit(-1); // exit program.
    }
}


// Create Spatial Map.
bool cameraControl::createSpatialMap(char *filePath, int count)
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
bool cameraControl::getSerialNumber(void)
{
    cout << endl;
    cout << "[Camera get] Getting serialnumber..." << endl;
    auto cameraInfos = getCameraInformation(); // Get ZED information
    cout << "[Camera get] ZED Serialnumber: " << cameraInfos.serial_number << endl;
    return true;
}


// Print Image data of the camera.
bool cameraControl::getImage(int count) 
{
	cout << endl;
    cout << "[Camera get] Getting Image..." << endl;

    int counter = 1; // counter for the whileloop
    Mat image;

	while (counter < count) // get image cout default: 50
    {
        // Grab an image
        if (grab(runtimeParams) == ERROR_CODE::SUCCESS) // check if there is an issue during capture
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
bool cameraControl::getDepth(int count)
{
	cout << endl;
    cout << "[Camera get] Getting Depth..." << endl;
    
    int counter = 0; // counter for the whileloop
    Mat image, depth, pointCloud;
 
    while (counter < count) // get images cout default: 50
    {   
        // Grab an image
        if (grab(runtimeParams) == ERROR_CODE::SUCCESS) // check if there is an issue during capture
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
bool cameraControl::getSensordata(void)
{
    if(grab() == ERROR_CODE::SUCCESS)
    {
        getSensorsData(sensorData, TIME_REFERENCE::CURRENT); // CURRENT: letimeStamp you retrieve the most recent sensor data available. || IMAGE: letimeStamp you retrieve the closest sensor data to the last image frame.
        printSensorData(); // function that prints the sensordata.
    }
    return true;
}


// Print Orientation, Acceleration, Velocity and Altitude.
bool cameraControl::printSensorData(void)
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
bool cameraControl::testWindow(void)
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

    // Setup for Opencv Viewer.
    auto textColor = CV_RGB(255, 255, 255); //wit
    auto textFont = cv::FONT_HERSHEY_TRIPLEX;
    auto textScaler = 0.7;
    auto textTickness = 1;
    auto lineType = cv::LINE_AA;
    auto textPosMid = (low_resolution.width/2)+10;
    
    // Keybinds.
    char key = ' ';
    //cout << "[Test Window] Keybinds: " << endl;
    //cout << "[Test Window] Press 'b' to begin SVO recording" << endl;
    //cout << "[Test Window] Press 's' to stop SVO recording" << endl;
    //cout << "[Test Window] Press 'q' to exit..." << endl;
    //cout << endl;
    
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
            
            // Create text input for resolution params
            string dispWindowResolution = "Window Resolution: " + std::to_string(low_resolution.height) + " X " + std::to_string(low_resolution.width);
            string dispCameraResolution = "Camera Resolution: " + std::to_string(resolution.height) + " X " + std::to_string(resolution.width);
            cv::putText(testWindow_ocv,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
            cv::putText(testWindow_ocv,dispCameraResolution,cv::Point(1150, 45),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
            
            // create text input for keybinds
            cv::putText(testWindow_ocv,"Press 'b' to begin SVO recording", cv::Point(10, 25),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(testWindow_ocv,"Press 's' to stop SVO recording", cv::Point(10, 50),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(testWindow_ocv,"Press 'q' to exit...", cv::Point(10, 75),textFont,textScaler,textColor,textTickness, lineType);

            // Create text input for position data.   
            getPosition(zedPose, REFERENCE_FRAME::WORLD); // Get the pose of the left eye of the camera with reference to the world frame  
            setTxt(zedPose.getOrientation(), textTranslation); // get orientation and put in in text
            setTxt(zedPose.getTranslation(), textOrientation); // get translation and put in text
            setTxt(zedPose.getEulerAngles(), textRotation); // get angle and put in text
            string dispTranslation = "Translation: " + string(textTranslation);
            string dispOrientation = "Orientation: " + string(textOrientation);
            string dispRotation =    "Rotation:    " + string(textRotation);
            cv::putText(testWindow_ocv,"Camera Pose:", cv::Point(10, 300),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(testWindow_ocv,dispTranslation, cv::Point(10, 325),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(testWindow_ocv,dispOrientation, cv::Point(10, 350),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(testWindow_ocv,dispRotation, cv::Point(10, 375),textFont,textScaler,textColor,textTickness, lineType);

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
            cv::putText(testWindow_ocv,"IMU Data:", cv::Point(textPosMid, 300),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(testWindow_ocv,dispImuOrientation, cv::Point(textPosMid, 325),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(testWindow_ocv,dispImuAcceleration, cv::Point(textPosMid, 350),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(testWindow_ocv,dispImuAngularVelocity, cv::Point(textPosMid, 375),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(testWindow_ocv,dispImuTemperature, cv::Point(textPosMid, 400),textFont,textScaler,textColor,textTickness, lineType);
            cv::putText(testWindow_ocv,dispAltitude, cv::Point(10, 400),textFont,textScaler,textColor,textTickness, lineType);


            // Viewer
            cv::namedWindow("Test Window"); // name the window
            cv::moveWindow("Test Window", 240,600); // move and fix viewer on the screen
            cv::imshow("Test Window", testWindow_ocv); // show the viewer
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
                
                // Create text input for resolution params
                string dispWindowResolution = "Window Resolution: " + std::to_string(low_resolution.height) + " X " + std::to_string(low_resolution.width);
                string dispSVOResolution = "SVO Resolution: " + std::to_string(resolution.height) + " X " + std::to_string(resolution.width);
                cv::putText(testWindow_ocv,dispWindowResolution,cv::Point(1150, 20),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
                cv::putText(testWindow_ocv,dispSVOResolution,cv::Point(1150, 45),textFont,0.5,textColor, textTickness, lineType); //target image //text //top-left position //font color
            
                // create text input for keybinds
                cv::putText(testWindow_ocv,"begon...",cv::Point(10, 25),textFont,textScaler,textColor, textTickness, lineType); //target image //text //top-left position //font color
                cv::putText(testWindow_ocv,"Press 's' to stop", cv::Point(10, 50),textFont,textScaler,textColor,textTickness, lineType);
                
                // create text input for frames recorded
                string dispFrames = "Frame # " + std::to_string(frames);
                cv::putText(testWindow_ocv,dispFrames, cv::Point(textPosMid, 20),textFont,textScaler,textColor,textTickness, lineType);

                // Create text input for position data.   
                getPosition(zedPose, REFERENCE_FRAME::WORLD); // Get the pose of the left eye of the camera with reference to the world frame  
                setTxt(zedPose.getOrientation(), textTranslation); // get orientation and put in in text
                setTxt(zedPose.getTranslation(), textOrientation); // get translation and put in text
                setTxt(zedPose.getEulerAngles(), textRotation); // get angle and put in text
                string dispTranslation = "Translation: " + string(textTranslation);
                string dispOrientation = "Orientation: " + string(textOrientation);
                string dispRotation =    "Rotation:    " + string(textRotation);
                cv::putText(testWindow_ocv,"Camera Pose:", cv::Point(10, 300),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(testWindow_ocv,dispTranslation, cv::Point(10, 325),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(testWindow_ocv,dispOrientation, cv::Point(10, 350),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(testWindow_ocv,dispRotation, cv::Point(10, 375),textFont,textScaler,textColor,textTickness, lineType);

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
                cv::putText(testWindow_ocv,"IMU Data:", cv::Point(textPosMid, 300),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(testWindow_ocv,dispImuOrientation, cv::Point(textPosMid, 325),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(testWindow_ocv,dispImuAcceleration, cv::Point(textPosMid, 350),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(testWindow_ocv,dispImuAngularVelocity, cv::Point(textPosMid, 375),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(testWindow_ocv,dispImuTemperature, cv::Point(textPosMid, 400),textFont,textScaler,textColor,textTickness, lineType);
                cv::putText(testWindow_ocv,dispAltitude, cv::Point(10, 400),textFont,textScaler,textColor,textTickness, lineType);
                
                // Display live stream
                cv::imshow("Test Window", testWindow_ocv);
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
    cout << endl;
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End :)     //////////////////////////////////////////////////////////////////////////////////////
