//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger Software
// Filename:    main.cpp
// Changed:     03-01-2023 | 14:35
//
// Discription: This file contains the main code for the unillogger software.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header Includes
#include "cameraController.h"
#include "cameraGrabber.h"
#include "dataProcessor.h"
#include "dataViewer.h"
#include "parseController.h"
#include "functionController.h"

// MISC Includes
#include <stdio.h>
#include <iostream>
#include <string>

// Using namespaces
using namespace sl;
using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Main     ////////////////////////////////////////////////////////////////////////////////////////////

bool nextPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

int main(int argc, char **argv) //  | argv[0]: run command | argv[1]: Function | argv[2]: path |
{
  //Parse controller.
  parseController parseController; // Create parse object.
  parseController.argCountChecker(argc, argv[1]); // Check for valid terminal command.
  parseController.argValueChecker(argc, argv[1]); // Check witch function needs to run.


  if (string(argv[1]) != "--logger" && string(argv[1]) != "-l")
  {
    // all old code from shift one. (Mainly for testing)
    parseController.runFunction(argv[2]);
  }
  else
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     Main Logger Code     ////////////////////////////////////////////////////////////////////////////////
    

    cameraController unilog;
    unilog.initCamera(); // initialize the ZED camera.
    unilog.initSVO(argv[2]); // initialize the SVO params.
    unilog.openCamera(); // open the ZED 2i.

    cameraGrabber gbr;
    gbr.startGrabThread();

    // init visualisation.
    dataViewer viewer;
    viewer.init();

    // loop throug svo file
    cout << "[main] press 'q' to stop the viewer" << endl;
    while (!viewer.viewerWasStopped()) // Loop until viewer catches the stop signal  original check: !viewer->wasStopped()
    {
      // data per frame
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr processedCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

      // get a new frame of the queue // stop when no data is available for a long time
      if(!nextPointcloud(inputCloud))
      {
        break;
      }

      {  
        // cout << "[main] start data proces" << endl;
        // dataProcessor pros;
        // pros.setInputCloud(inputCloud);
        // pros.processCloud();
        // pros.getProcessedCloud(processedCloud);
      }

      //update Point cloud
      viewer.update(inputCloud);
    }

    viewer.close(); // Close the viewer
    gbr.stopGrabThread(); // Stop the tread
    unilog.closeCamera(); // close the camera
    
    cout << "[main] thread has been stopped, end of program" << endl;
    while(true){} // keep waiting 
  }
}


bool nextPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  cout << "[main::nextpointcloud] pop next pointcloud.." << endl;
  thut::sharedGrabData_s pulledData;

  int counter = 0;
  while(thut::grabQueue.IsEmpty())
  {
    counter++;
    if(counter > 5000)
    {
      cout << "[main::nextpointcloud] Long time no data available!, exiting" << endl;
      return false;
    }
  }
  pulledData = thut::grabQueue.PopBack();


  thut::grabQueue.Clear();


  cloud = pulledData.pointcloud;
  return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////