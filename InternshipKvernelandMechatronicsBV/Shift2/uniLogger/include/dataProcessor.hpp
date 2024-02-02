#ifndef DATAPROCESSOR_H
#define DATAPROCESSOR_H

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurma
// Date:        15-11-2023
// Project:     Unilogger Software
// Filename:    dataProcessor.hpp
// Changed:     18-01-2024 | 10:01
//
// Discription: This file contains the class to proces all data.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header includes
#include <pcl/filters/statistical_outlier_removal.h>
#include "filter.hpp"
#include "planeController.hpp"
#include "logCout.h"
#include "utilities.hpp"

// MISC Includes
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <cmath>
#include <chrono>
#include <vector>
#include <Eigen/Dense>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Class: dataProcessor      ///////////////////////////////////////////////////////////////////////////

// class to procces data.
class dataProcessor
{
  private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr insertedCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_transformedCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_croppedCloud;


  public:
    dataProcessor(void);
    ~dataProcessor(void);

    void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud);
    void processCloud(void);
    void getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputCloud);
    void filterCloud(void);
    void projectPlane(void);
    void cropPointCloud(float width, float distance, float depth);
};


#endif // DATA_PROCESSOR_H


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////