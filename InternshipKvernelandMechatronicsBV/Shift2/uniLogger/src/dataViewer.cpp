//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurma
// Date:        15-11-2023
// Project:     Unilogger Software
// Filename:    dataViewer.cpp
// Changed:     04-01-2023 | 15:10
//
// Discription: This file contains all member functions from the dataViewer class.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header Includes
#include "dataViewer.hpp"
#include "cameraController.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Function     ////////////////////////////////////////////////////////////////////////////////////////

// class constructor
dataViewer::dataViewer(void)
{
  pclCloud = nullptr;
  pclViewer = nullptr;
  svoNbFrame = 0;
}


// class destructor
dataViewer::~dataViewer(void)
{
  pclCloud = nullptr;
}


// viewer init
void dataViewer::init(void)
{
  // Open 3D viewer and add point cloud
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Unilogger Pointcloud Viewer"));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> toRGB(pclCloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr emptyPointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pclViewer = viewer;
  pclViewer->addPointCloud<pcl::PointXYZRGB>(emptyPointCloud, toRGB);
  //pclViewer->setBackgroundColor(0.12, 0.12, 0.12); // gray
  //pclViewer->setBackgroundColor(0.02, 0.02, 0.02); // black
  //pclViewer->setBackgroundColor(0.255, 0.255, 0.255); // white
  pclViewer->setBackgroundColor(0.0, 0.0, 0.2); // dark navy
  pclViewer->addCoordinateSystem(0.1);
  pclViewer->initCameraParameters ();
  pclViewer->setCameraPosition(0, -2, 0.88,    0, 0.35, 0.22,   0, 0.55, 0.84);
  pclViewer->addText("PCL Pointcloud Visualizer for the Unilogger.",225,475,25,10,10,10,"title",0);
  svoNbFrame = zed.getSVONumberOfFrames();
  std::string nbFrame = "Frame #" + std::to_string(svoNbFrame);
  pclViewer->addText(nbFrame,5,20,18,10,10,10,"nbFrame",0);
  Log() << "[dataViewer::init] viewer initialized\n";
}


// update the viewer
void dataViewer::update(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  if(cloud == nullptr)
  {
    Log() << "[dataViewer::update] update viewer\n";
    return;
  }
  pclCloud = cloud;
  pclViewer->updatePointCloud(pclCloud);
}


// check if viewer is stopped
bool dataViewer::stopped()
{
  Log() << "[dataViewer::viewerWasStopped] spin viewer\n";
  pclViewer->spinOnce(5);
  return pclViewer->wasStopped();
}


void dataViewer::close(void)
{
  Log() << "[dataViewer::close] close viewer\n";
  pclViewer->close();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////