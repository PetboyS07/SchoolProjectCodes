//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurma
// Date:        15-11-2023
// Project:     Unilogger Software
// Filename:    dataProcessor.cpp
// Changed:     03-01-2023 | 14:35
//
// Discription: This file contains all member functions from the dataProcessor class.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header Includes
#include "dataProcessor.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Function     ////////////////////////////////////////////////////////////////////////////////////////

// constructor
dataProcessor::dataProcessor(void)
{
    inputProcessCloud = nullptr;
    processedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    filteredCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
}


// destructor
dataProcessor::~dataProcessor(void)
{
    inputProcessCloud = nullptr;
    processedCloud = nullptr;
    filteredCloud = nullptr;
}


// get the cloud that needs to be filterd.
void dataProcessor::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud)
{
    inputProcessCloud = inputCloud; // set input cloud
    cout << "[dataProcessor::setInputCloud] inputCloud set" << endl;
}


// perform all processes on the cloud.
void dataProcessor::processCloud(void)
{
    filterCloud();
    processedCloud = filteredCloud;

    cout << "[dataProcessor::processCloud] cloud processed." << endl;
}


// return the processed cloud.
void dataProcessor::getProcessedCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputCloud)
{
    if(processedCloud == nullptr)
    {
        std::cout << "[Processing]: Cannot get processed cloud, process the data first!" << std::endl;
        return;
    }

    // pass through the pointer address
    outputCloud = processedCloud;
    cout << "[dataProcessor::getProcessedCloud] sending cloud.." << endl;
}


// apply a statisticalOutlier filter to cloud.
void dataProcessor::filterCloud(void)
{
    // cout << "[dataProcessor::filterCloud] start cloud filter" << endl;
    // // Create the filtering object
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    // sor.setInputCloud(inputProcessCloud); // specify the pointcloud to filter
    // cout << "[dataProcessor::filterCloud] input cloud set" << endl;
    // sor.setMeanK(50); // Number of neighbors to analyse fo each point
    // cout << "[dataProcessor::filterCloud] mean set" << endl;
    // sor.setStddevMulThresh(1.0); // Standard deviation multiplier,
    // cout << "[dataProcessor::filterCloud] std set" << endl;
    // sor.filter(*filteredCloud); // al point larger than sd of the mean distance to the query point whill be marked as outliers and removed.
    // cout << "[dataProcessor::filterCloud] end cloud filter" << endl;

    filteredCloud = inputProcessCloud;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////