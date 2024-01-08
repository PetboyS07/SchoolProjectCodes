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
  insertedCloud = nullptr;
  filteredCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
}


// class destructor
dataProcessor::~dataProcessor(void)
{
  insertedCloud = nullptr;
  filteredCloud = nullptr;
}


//Function to set the inputcloud wich need to be proccesed.
void dataProcessor::setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud)
{
  if(!inputCloud->isOrganized())
  {
    std::cout << "[Processing]: The input cloud is not organised!" << std::endl;
    insertedCloud = nullptr;
    return;
  }
  insertedCloud = inputCloud;
}


//Function to perform all processes to the cloud.
void dataProcessor::processCloud(void)
{
  if(insertedCloud == nullptr)
  {
    std::cout << "[Processing]: there is no cloud to process." << std::endl;
    return;
  }

  filterCloud();
}


// Function to recieve the processed cloud.
void dataProcessor::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputCloud)
{
  if(filteredCloud == nullptr)
  {
    std::cout << "[Processing]: there is no cloud to get" << std::endl;
    return;
  }

  // pass through the pointer address.
  outputCloud = filteredCloud;
}


// Function to filter the cloud for outliers.
void dataProcessor::filterCloud(void)
{
  // GPU Accelerated radius outlier search :)
  // Using radius filter parameters based on HD2K format
  double searchRadius = 0.09f;
  int minNeighbors = 25;
  // Using dummy pcl ptr for the noise cloud --> smart pointer that will automatically deallocate the memory when out of scope
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr trashcanCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  RadiusOutlierRemoval ror;
  ror.SetInputCloud(insertedCloud);
  ror.SetRadiusSearch(searchRadius);
  ror.SetMinNeighborsInRadius(minNeighbors);
  ror.Filter(filteredCloud, trashcanCloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////