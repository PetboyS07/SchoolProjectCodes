#ifndef RADIUS_OUTLIER_REMOVAL_H
#define RADIUS_OUTLIER_REMOVAL_H


#include <math.h>
#include <memory>
#include <iostream>
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>

#include <pcl/io/io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>


class RadiusOutlierRemoval
{
  public:

    RadiusOutlierRemoval(void);

    void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud);

    void SetRadiusSearch(double radius);
    void SetMinNeighborsInRadius(int nrOfPoints);


    // filter the input cloud and put the resulted cloud in the output cloud and noise in the noiseCloud if specified
    void Filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr noiseCloud);



  private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_inCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_convertedCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_filteredCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_noiseCloud;
    std::vector<pcl::PointXYZ> m_searchQuery;
    std::vector<float> m_radiusSizes;
    std::vector<int> m_outpuSizesRadiusSearch;
    //std::vector<int> m_outputDataRadiusSearch; // data not needed since only amount of points is necessary.
    double m_searchRadius;
    double m_minAmountOfRequiredPoints;
    bool m_copyUnvalidPoints;

    void CopyPointStructure(void);
    void SetOctreeParameterData(void);
    void PerformOctreeSearch(void);
    void FilterCloud(void);


    bool CheckPointIsValid(pcl::PointXYZRGB &point);
};

#endif //RADIUS_OUTLIER_REMOVAL_H


