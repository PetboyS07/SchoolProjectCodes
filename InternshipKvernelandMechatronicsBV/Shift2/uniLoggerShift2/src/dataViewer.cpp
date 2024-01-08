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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Function     ////////////////////////////////////////////////////////////////////////////////////////

// class constructor
dataViewer::dataViewer(void)
{
  pclCloud = nullptr;
  pclViewer = nullptr;
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
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Live Seedbed viewer"));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> toRGB(pclCloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr emptyPointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pclViewer = viewer;
  pclViewer->addPointCloud<pcl::PointXYZRGB>(emptyPointCloud, toRGB);
  //pclViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
  //pclViewer->setBackgroundColor(0.12, 0.12, 0.12); // gray
  pclViewer->setBackgroundColor(0.02, 0.02, 0.02); // black
  pclViewer->addCoordinateSystem(0.1);
  pclViewer->initCameraParameters ();
  pclViewer->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
  pclViewer->setCameraClipDistances(0.1,1000);

  std::cout << "[dataViewer::init] viewer initialized" << std::endl;
}


// update the viewer
void dataViewer::update(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  if(cloud == nullptr)
  {
    std::cout << "[dataViewer::update] update viewer" << std::endl;
    return;
  }
  pclCloud = cloud;
  pclViewer->updatePointCloud(pclCloud);
}


// check if viewer is stopped
bool dataViewer::stopped()
{
  std::cout << "[dataViewer::viewerWasStopped] spin viewer" << std::endl;
  pclViewer->spinOnce(5);
  return pclViewer->wasStopped();
}

void dataViewer::close(void)
{
  std::cout << "[dataViewer::close] close viewer" << std::endl;
  pclViewer->close();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////