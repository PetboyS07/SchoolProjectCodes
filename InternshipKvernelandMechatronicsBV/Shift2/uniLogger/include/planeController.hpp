#ifndef PLANE_CONTROLLER_H
#define PLANE_CONTROLLER_H

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Unkown
// Date:        01-2024
// Project:     UniLogger
// Filename:    planeController
// Changed:     22-01-2024
//
// Discription: Class to control plane transformation.

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <cmath>
#include <iostream>
//#include <limits>

#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#define _USE_MATH_DEFINES


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Class:planeController     ///////////////////////////////////////////////////////////////////////////

class planeController
{
  public:
    planeController(void);
    void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud);
    void SetAveragingIndexWidth(int width);
    void SetMinPointsValidLine(int nrOfPoints);
    void Transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud);

  private:
    struct barData
    {
      std::vector<double> barLengths;
      std::vector<pcl::PointXYZRGB> validBarPoints;
      double averageX;
      double averageY;
      double averageZ;
      double medianBarLength;

    } mBar1Data, mBar2Data;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformationCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud;
    int mMinPointsValidLine;
    int mIndexSearchWidth;
    float planeHeightZ;
    float mPlaneAngle;
    bool mNeedFlipping;
    pcl::PointXYZ mPlaneTransl;
    bool CalculateBar1Point(void);
    bool CalculateBar2Point(void);
    void CalculatePlaneAngle(void);
    void CalculatePlaneTranslation(void);
    void TransformPlaneToMid(void);
    double CalculateMedianValue(std::vector<double> vect, int size);
    void CalculateAverageCoordinates(barData &bar);
    bool CheckPointIsValid(pcl::PointXYZRGB* point);
};

#endif //PLANE_CONTROLLER_H


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     End ;)     //////////////////////////////////////////////////////////////////////////////////////////