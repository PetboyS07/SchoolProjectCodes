//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger software
// Filename:    cameraGrabber.cpp
// Changed:     04-01-2024 | 14:49
//
// Discription: This file contains all memberfunctions from the cameraGrabber class.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header Includes
#include "cameraGrabber.h"


// parameters
std::unique_ptr<std::thread> thread = nullptr;
std::atomic<bool> runThread = false;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointcloud;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Functions     ///////////////////////////////////////////////////////////////////////////////////////


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
void cameraGrabber::runGrabThread(void)
{
  std::cout << "[cameraGrabber::runGrabThread] Start grab..." << std::endl;

  while(runThread)
  {
    // get the camera pointcloud and check if there are no errors
    if(!getSlPointcloud())
    {
      runThread = false;            
      std::cout << "[cameraGrabber::runGrabThread] No Pointcloud, runThread = false;" << std::endl;
    }

    if(!transformPointcloud()){continue;}

    pushData();
  }

  std::cout << "[cameraGrabber::runGrabThread] Stop grab..." << std::endl;
}


// This function frees the ZED, its callback(thread) and the viewer.
void cameraGrabber::stopGrabThread(void)
{
  runThread = false;
  thread->join();
  thread = nullptr;
}


// This function loops trough the pointcloud to transform all points from sl:: format to pcl:: format.
bool cameraGrabber::transformPointcloud(void)
{
  std::cout << "[CameraGrabber::transformPointcloud] start transforming pointcloud." << std::endl;
  
  // create a new empty pointcloud en allocate the resolution.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->width = zed.getCameraInformation().camera_resolution.width;
  cloud->height = zed.getCameraInformation().camera_resolution.height;
  cloud->resize(cloud->width * cloud->height);

  // get and put the slpointcloud in pionter.
  float *p_data_cloud = slPointcloud.getPtr<float>(sl::MEM::CPU);

  // check if there is actually something in the pointer.
  if(p_data_cloud == nullptr)
  {
    std::cerr << "[CameraGrabber::transformPointcloud] No pointcloud to transform." << std::endl;
    return false;
  }

  int unvalidMeasurements = 0;
  int index = 0;

  // loop trough all points and transform
  //std::cout << "[CameraGrabber::transformPointcloud] adjusting..." << std::endl;
  for (auto &it : cloud->points)
  {
    float X = p_data_cloud[index];
    if (!isValidMeasure(X)) 
    {
        it.x = it.y = it.z = it.rgb = 0;
        unvalidMeasurements++;
    }
    else
    {
        it.x = X;
        it.y = p_data_cloud[index + 1];
        it.z = p_data_cloud[index + 2];
        it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format it.z*(0xC0F9D9);
    }
      index += 4;

  }

  // save the pointer
  pclPointcloud = cloud;
  return true;
}

//This function convert a RGBA color packed into a packed RGBA PCL compatible format.
inline float cameraGrabber::convertColor(float colorIn) 
{
    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float *> (&color_uint);
}


// this function pushes all data to the queue.
void cameraGrabber::pushData(void)
{
  std::cout << "[CameraGrabber::transformPointcloud] Pushing data..." << std::endl;
  thut::sharedGrabData_s data;
  data.frame = zed.getSVOPosition();
  data.pointcloud = pclPointcloud;

  pclPointcloud = nullptr;
  thut::grabQueue.PushBack(data);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////