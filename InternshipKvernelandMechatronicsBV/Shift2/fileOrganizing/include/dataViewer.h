#ifndef DATA_VIEWER_H
#define DATA_VIEWER_H


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger software
// Filename:    dataViewer.h
// Changed:     03-01-2023 | 14:35
//
// Discription: This file contains the class to view al kinds of data.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// PCL includes
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

// MISC Includes
#include <stdio.h>
#include <iostream>
#include <string>

// Using namespaces
using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Class: dataViewer      //////////////////////////////////////////////////////////////////////////////

shared_ptr<pcl::visualization::PCLVisualizer> createRGBViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud); 

// class to view data
class dataViewer
{
    private:
    public:
        dataViewer(void);
        ~dataViewer(void);

        std::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud;

        void init(void);
        void update(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        bool viewerWasStopped(void);
        void close(void);

        bool singlePclViewer(char* filepath);
        bool filterCompareViewer(char* filepath);
};


#endif // DATA_VIEWER_H


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////