//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger software
// Filename:    cameraGrabber.cpp
// Changed:     03-01-2023 | 14:35
//
// Discription: This file contains all memberfunctions from the cameraGrabber class.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header Includes
#include "cameraGrabber.h"

using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Funtions     ////////////////////////////////////////////////////////////////////////////////////////


// Constructor for the cameraGrabber class.
cameraGrabber::cameraGrabber(void)
{
    runThread = false;
    thread = nullptr;
}


// Destructor for the cameraGrabber class.
cameraGrabber::~cameraGrabber(void)
{
    // Code

}


// This functions start the ZED's thread that grab images and data.
void cameraGrabber::startGrabThread(void) 
{
    if(thread != nullptr)
    {
    runThread = false;
    thread->join();
    }

    // start the data grabber/conversion thread
    runThread = true;
    thread = std::make_unique<std::thread>(&cameraGrabber::runGrabThread, this);
}


//This function loops to get the point cloud from the ZED. It can be considered as a callback.
void cameraGrabber::runGrabThread() 
{
    cout << "[cameraGrabber::runGrabThread] Start grab..." << endl;

    while(runThread)
    {
        // get the camera pointcloud and check if there are no errors
        if(!camcon.getSlPointcloud())
        {
            runThread = false;
            cout << "[cameraGrabber::runGrabThread] No Pointcloud, runThread = false;" << endl;
        }

        if(!transformPointcloud()){continue;}

        pushData();
    }

    cout << "[cameraGrabber::runGrabThread] Stop grab..." << endl;
}


// This function frees the ZED, its callback(thread) and the viewer.
void cameraGrabber::stopGrabThread(void) 
{
    runThread = false;
    thread->join();
    thread = nullptr;
}

bool cameraGrabber::transformPointcloud(void)
{
    cout << "[CameraGrabber::transformPointcloud] start transforming pointcloud." << endl;
    
    // create a new empty pointcloud en allocate the resolution.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = zed.getCameraInformation().camera_resolution.width;
    cloud->height = zed.getCameraInformation().camera_resolution.height;
    cloud->resize(cloud->width * cloud->height);

    // get the pointer of the sl pointcloud.
    float *p_data_cloud = slPointcloud.getPtr<float>(sl::MEM::CPU);

    // check for NULL ptr
    if(p_data_cloud == nullptr)
    {
        cerr << "[CameraGrabber::transformPointcloud] No pointcloud to transform." << endl;
        return false;
    }

    int unvalidMeasurementsCounter = 0;
    int index = 0;

    // Check and adjust points for PCL format
    cout << "[CameraGrabber::transformPointcloud] adjusting..." << endl;
    for (auto &it : cloud->points)
    {
        float X = p_data_cloud[index];
        if (!isValidMeasure(X)) // Checking if it's a valid point
        {
            //cout << "[CameraGrabber::transformPointcloud] Unvalid measurement!" << endl;
            it.x = it.y = it.z = it.rgb = 0;
            unvalidMeasurementsCounter++;
        }
        else
        {
            //cout << "[CameraGrabber::transformPointcloud] Valid measurement!" << endl;
            it.x = X;
            it.y = p_data_cloud[index + 1];
            it.z = p_data_cloud[index + 2];


            //cout << "[CameraGrabber::transformPointcloud] xyz, rgb: " << it.x << ", " << it.y << ", " << it.z << ", " << it.rgb << endl;
            it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format it.z*(0xC0F9D9);
            //cout << "[CameraGrabber::transformPointcloud] xyz, rgb: " << p_data_cloud[index + 0] << ", " << p_data_cloud[index + 1] << ", " << p_data_cloud[index + 2] << ", " << p_data_cloud[index + 3] << endl;
        }
        index += 4;
    }

    // save the pointcloud
    pclPointcloud = cloud;
    cout << "[CameraGrabber::transformPointcloud] pointcloud saved" << endl;
    return true;
}


// This function convert a RGBA color packed into a packed RGBA PCL compatible format.
inline float cameraGrabber::convertColor(float colorIn) 
{
    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float *> (&color_uint);
}


bool cameraGrabber::pushData(void)
{
    cout << "[CameraGrabber::transformPointcloud] Pushing data..." << endl;

    thut::sharedGrabData_s data;
    data.frame = zed.getSVOPosition();
    data.pointcloud = pclPointcloud;

    pclPointcloud = nullptr;
    thut::grabQueue.PushBack(data);
}





//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////