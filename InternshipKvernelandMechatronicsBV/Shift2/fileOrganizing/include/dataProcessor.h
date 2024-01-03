#ifndef DATA_PROCESSOR_H
#define DATA_PROCESSOR_H


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger software
// Filename:    dataProcessor.h
// Changed:     03-01-2023 | 14:35
//
// Discription: This file contains the class to proces all data.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// MISC Includes
#include <stdio.h>
#include <iostream>
#include <string>

// PCL includes
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

// Using namespaces
using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Class: dataProcessor      ///////////////////////////////////////////////////////////////////////////

// class to procces data.
class dataProcessor
{
    private:

    public:
        dataProcessor(void);
        ~dataProcessor(void);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputProcessCloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr processedCloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud;


        void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud);
        void processCloud(void);
        void getProcessedCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputCloud);
        void filterCloud(void);
        
};


#endif // DATA_PROCESSOR_H


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////