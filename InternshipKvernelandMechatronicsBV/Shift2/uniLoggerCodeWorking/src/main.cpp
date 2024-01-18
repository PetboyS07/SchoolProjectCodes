//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger Software
// Filename:    main.cpp
// Changed:     04-01-2023 | 09:45
//
// Discription: This file contains the main code for the unilogger software.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header Includes
#include "threadUtilities.hpp"
#include "cameraController.h"
#include "cameraGrabber.h"
#include "dataProcessor.hpp"
#include "dataViewer.hpp"
#include "parseController.h"
#include "logCout.h"
#include "functionController.h"

// MISC Includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

// Using namespaces
using namespace sl;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Main     ////////////////////////////////////////////////////////////////////////////////////////////

bool nextPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
int svopos = 0;

int main(int argc, char **argv)
{
  //Parse controller.
  // ------------------------------------------------------------------------------------
  parseController parseController; // Create parse object.
  parseController.argCountChecker(argc, argv[1]); // Check for valid terminal command.
  parseController.argValueChecker(argc, argv[1]); // Check witch function needs to run.
  // ------------------------------------------------------------------------------------

  if (string(argv[1]) != "--logger" && string(argv[1]) != "-l")
  {
    // all old code from shift one. (Mainly for testing)
    parseController.runFunction(argv[2]);
  }
  else
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///     Main Logger Code     ////////////////////////////////////////////////////////////////////////////////
    

    // Setup the camera.
    // -----------------------------------------------------------------
    initCamera();             // initialize the ZED camera.

    if(string(argv[2]) != "--live")
    {
      initSVO(argv[2]);         // initialize the SVO params.
    }

    openCamera();             // open the ZED 2i.
    // -----------------------------------------------------------------
    

    // setup the thread Grabber.
    // -----------------------------------------------------------------
    cameraGrabber grabr;      // Create object.
    grabr.startGrabThread();  // start the data grabber thread.
    // -----------------------------------------------------------------


    // setup the dataViewer.
    // -----------------------------------------------------------------
    dataViewer viewer;        // Create object.
    viewer.init();            // initialize viewer.
    // -----------------------------------------------------------------

    // loop through the pointcloud
    while(!viewer.stopped())
    {
      // Setup pointclouds
      // ----------------------------------------------------------------
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudToProcess;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr processedcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      // ----------------------------------------------------------------


      // get a new frame of the queue // stop when no data is available for a long time
      // ----------------------------------------------------------------
      if(!nextPointcloud(cloudToProcess))
      {
        break;
      }
      // ----------------------------------------------------------------


      // process the data
      // ---------------------------------------------------------------
      dataProcessor proces;             // setup
      proces.setCloud(cloudToProcess);  // input
      proces.processCloud();            // process
      proces.getCloud(processedcloud);  // output
      // ---------------------------------------------------------------
      
      // Save pcl file
      // ---------------------------------------------------------------
      //string pcdpath = "../../pcl/pclfromsvo/pcd_" + to_string(svopos) + ".pcd";
      pcl::io::savePCDFileASCII ("pcl_210/pcl_" + to_string(svopos) + ".pcd", *processedcloud);      
      // ---------------------------------------------------------------


      // View the data
      // ---------------------------------------------------------------
      viewer.update(processedcloud);    // update cloud
      // ---------------------------------------------------------------
    }


    // Stop the data grabber thread and close the camera
    //  ----------------------------------------------------------------
    grabr.stopGrabThread();
    closeCamera();
    cout << "[main] thread has been stopped, end of program" << endl;
    //  ----------------------------------------------------------------
    

    //Timeout timer.
    //  ----------------------------------------------------------------
    int count = 0;
    while(count < 999999)
    {
      count++;
    }
    Log() << "[main] closing program..\n";
    //  ----------------------------------------------------------------
  }
}


bool nextPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  Log() << "[main::nextpointcloud] pop next pointcloud..\n";
  thut::sharedGrabData_s pulledData;

  int counter = 0;
  while(thut::grabQueue.IsEmpty())
  {
    counter++;
    if(counter > 5000)
    {
      Log() << "[main::nextpointcloud] No data detected for a long time.\n";
      return false;
    }
  }

  pulledData = thut::grabQueue.PopBack();
  thut::grabQueue.Clear();
  cloud = pulledData.pointcloud;
  svopos = pulledData.frame;
  return true;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////