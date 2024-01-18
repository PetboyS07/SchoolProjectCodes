#ifndef DATA_VIEWER_H
#define DATA_VIEWER_H

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger software
// Filename:    dataViewer.h
// Changed:     04-01-2023 | 14:53
//
// Discription: This file contains the class to view al kinds of data.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

<<<<<<<< HEAD:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerCodeWorking/include/dataViewer.hpp
// Header Includes
#include "logCout.h"

// PCL Includes
========
// PCL includes
>>>>>>>> 71cccea26b3de34f671a0b0d3ea02f86ae34e0e1:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerShift2/include/dataViewer.hpp
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/console/parse.h>

// MISC Includes
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <chrono>
#include <vector>
#include <Eigen/Dense>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Class: dataViewer      //////////////////////////////////////////////////////////////////////////////


// class to view data
class dataViewer
{
  private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> m_rgbHandler;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud;
<<<<<<<< HEAD:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerCodeWorking/include/dataViewer.hpp
    int svoNbFrame;
========
>>>>>>>> 71cccea26b3de34f671a0b0d3ea02f86ae34e0e1:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerShift2/include/dataViewer.hpp

  public:
    dataViewer(void);
    ~dataViewer(void);
    void init(void);
    void update(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    bool stopped(void);
    void close(void);
<<<<<<<< HEAD:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerCodeWorking/include/dataViewer.hpp
========

>>>>>>>> 71cccea26b3de34f671a0b0d3ea02f86ae34e0e1:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerShift2/include/dataViewer.hpp
};

#endif //DATA_VIEWER_H

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////