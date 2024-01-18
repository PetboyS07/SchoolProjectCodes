//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurma
// Date:        15-11-2023
// Project:     Unilogger Software
// Filename:    dataProcessor.cpp
// Changed:     04-01-2023 | 16:09
//
// Discription: This file contains all member functions from the dataProcessor class.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header Includes
#include "dataProcessor.hpp"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Function     ////////////////////////////////////////////////////////////////////////////////////////

// class constructor
dataProcessor::dataProcessor(void)
{
  Log() << "[dataProcessor::dataProcessor] Start constructor\n";
  insertedCloud = nullptr;
  filteredCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
}


// class destructor
dataProcessor::~dataProcessor(void)
{
  Log() << "[dataProcessor::~dataProcessor] Start desctructor\n";
  insertedCloud = nullptr;
  filteredCloud = nullptr;
}


//Function to set the inputcloud wich need to be proccesed.
void dataProcessor::setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud)
{
  if(!inputCloud->isOrganized())
  {
    Log() << "[Processing]: The input cloud is not organised!\n";
    insertedCloud = nullptr;
    return;
  }
  insertedCloud = inputCloud;
  Log() << "[dataProcessor::setCloud] Inputcloud Set\n";
}


//Function to perform all processes to the cloud.
void dataProcessor::processCloud(void)
{
  if(insertedCloud == nullptr)
  {
    Log() << "[Processing]: there is no cloud to process.\n";
    return;
  }

  filterCloud();
  Log() << "[dataProcessor::processCloud] Processes finisched.\n";
}


// Function to recieve the processed cloud.
void dataProcessor::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputCloud)
{
  if(filteredCloud == nullptr)
  {
    Log() << "[Processing]: there is no cloud to get\n";
    return;
  }

  // pass through the pointer address.
  outputCloud = filteredCloud;
  Log() << "[dataProcessor::getCloud] OutputCloud Set\n";
}


// Function to filter the cloud for outliers.
void dataProcessor::filterCloud(void)
{
  // pointer to trow away outliers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr trashcanCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  // // 1. arable land | filter setup 
  // double radius = 0.02; // Radius of the sphere we check. default: 0.05
  // int numberOfPoints = 150; // Minimum amount of neighbors points inside the radius. default: 16

  // 2. algoritm | filter setup 
  double radius = 0.05; // Radius of the sphere we check. default: 0.05
  int numberOfPoints = 16; // Minimum amount of neighbors points inside the radius. default: 16

  // // 3. halway | filter setup 
  // double radius = 0.05; // Radius of the sphere we check. default: 0.05
  // int numberOfPoints = 100; // Minimum amount of neighbors points inside the radius. default: 16

  // Radius outlier removal
  RadiusOutlierRemoval ror;
  ror.SetInputCloud(insertedCloud);
  ror.SetRadiusSearch(radius);
  ror.SetMinNeighborsInRadius(numberOfPoints);
  ror.Filter(filteredCloud, trashcanCloud);
  Log() << "[dataProcessor::filterCloud] Cloud Filterd\n";
  // Statistical outlier removal
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  // sor.setInputCloud (insertedCloud); // specify the pointcloud to filter
  // sor.setMeanK (50); // Number of neighbors to analyse fo each point
  // sor.setStddevMulThresh (1.0); // Standard deviation multiplier,
  // sor.filter (*filteredCloud); // al point larger than sd of the mean distance to the query point whill be marked as outliers and removed.
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////