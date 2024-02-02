#include "filter.hpp"


RadiusOutlierRemoval::RadiusOutlierRemoval(void)
{
  m_searchRadius = 0;
  m_minAmountOfRequiredPoints = 0;
  m_copyUnvalidPoints = 0;
  m_convertedCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  m_filteredCloud = nullptr;
  m_noiseCloud = nullptr;
  //if (!m_convertedCloud) m_convertedCloud->get()->points.clear();
}


void RadiusOutlierRemoval::SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud)
{
  if(inputCloud == nullptr)
  {
    m_inCloud = nullptr;
    std::cout << "[RadiusOutlierRemoval]: No input pointcloud is specified, exiting!" << std::endl;
    return;
  }
  m_inCloud = inputCloud;
}


void RadiusOutlierRemoval::SetRadiusSearch(double radius)
{
  if(radius < 0)
  {
    m_searchRadius = 0;
    std::cout << "[RadiusOutlierRemoval]: Only a positive radius can be specified, exiting!" << std::endl;
    return;
  }

  m_searchRadius = radius;
}


void RadiusOutlierRemoval::SetMinNeighborsInRadius(int nrOfPoints)
{
  if(nrOfPoints < 1)
  {
    m_minAmountOfRequiredPoints = 0;
    std::cout << "[RadiusOutlierRemoval]: Minimum of 1 neighbor has to be set, exiting!" << std::endl;
    return;
  }

  // set the min amount of neighbors + the point itself
  m_minAmountOfRequiredPoints = nrOfPoints + 1;
}


void RadiusOutlierRemoval::Filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr noiseCloud)
{
  // check if required conditions are met
  if(outputCloud == nullptr)
  {
    std::cout << "[RadiusOutlierRemoval]: No input cloud was submitted, exiting!" << std::endl;
    return;
  }
  if(m_searchRadius == 0)
  {
    std::cout << "[RadiusOutlierRemoval]: Search radius was not set up correctly, exiting!" << std::endl;
    return;
  }
  if(m_minAmountOfRequiredPoints == 0)
  {
    std::cout << "[RadiusOutlierRemoval]: Minimum amount of neighbors was not set up correctly, exiting!" << std::endl;
    return;
  }

  // check if optional noise cloud pointer is specified
  if(noiseCloud != nullptr)
  {
    m_copyUnvalidPoints = 1;
    m_noiseCloud = noiseCloud;
  }

  m_filteredCloud = outputCloud;

  //std::cout << "start structure copy!" << std::endl;
  CopyPointStructure();
  //std::cout << "point structures copied!" << std::endl;

  SetOctreeParameterData();
  //std::cout << "Octree parameters set!" << std::endl;

  PerformOctreeSearch();
  //std::cout << "Octree gpu operation done!" << std::endl;

  FilterCloud();
  //std::cout << "Octree cloud filtering done!" << std::endl;
}


void RadiusOutlierRemoval::CopyPointStructure(void)
{

    // duplicate cloud for the output data
    m_filteredCloud->width = m_inCloud->width;
    m_filteredCloud->height = m_inCloud->height;
    m_filteredCloud->is_dense = m_inCloud->is_dense;
    m_filteredCloud->points.resize(m_filteredCloud->width * m_filteredCloud->height);
    pcl::copyPointCloud(*m_inCloud, *m_filteredCloud);

  // duplicate cloud with <PointXYZ> structure --> only this type of point structure is supported by the GPU octree class
  m_convertedCloud->width = m_inCloud->width;
  m_convertedCloud->height = m_inCloud->height;
  m_convertedCloud->is_dense = m_inCloud->is_dense;
  m_convertedCloud->points.resize(m_convertedCloud->width * m_convertedCloud->height);
  pcl::copyPointCloud(*m_inCloud, *m_convertedCloud);

  //std::cout << "copy1" << std::endl;

  if(m_copyUnvalidPoints)
  {
    m_noiseCloud->width = 1;
    m_noiseCloud->is_dense = false;
  }

  // resize the query because it has multiple data variables
  m_searchQuery.resize(m_convertedCloud->width * m_convertedCloud->height);
}


void RadiusOutlierRemoval::SetOctreeParameterData(void)
{
  // set the querry to do a radius search for every point in the cloud with the specified radius
  const int size = static_cast<const int>(m_inCloud->width * m_inCloud->height);
  for(int k = 0; k < size; k++)
  {
    m_searchQuery[k].x = m_convertedCloud->at(k).x;
    m_searchQuery[k].y = m_convertedCloud->at(k).y;
    m_searchQuery[k].z = m_convertedCloud->at(k).z;
    m_radiusSizes.push_back(m_searchRadius);
  }
}


void RadiusOutlierRemoval::PerformOctreeSearch(void)
{
  // initialize memory on the d(evice)->GPU and copy pointcloud and octree data from the h(ost)->CPU to the device
  pcl::gpu::Octree::PointCloud d_cloud;
  d_cloud.upload(m_convertedCloud->points);

  pcl::gpu::Octree::Queries d_searchQuery;
  d_searchQuery.upload(m_searchQuery);

  pcl::gpu::Octree::Radiuses d_radiusSizes;
  d_radiusSizes.upload(m_radiusSizes);

  pcl::gpu::NeighborIndices d_result(d_searchQuery.size(), m_minAmountOfRequiredPoints);

  // initialize the octree on the device
  pcl::gpu::Octree d_octree;
  d_octree.setCloud(d_cloud);
  d_octree.build();

  // perform the radius search with the specified query and radius, specify the max amount of storable answers and save the results
  d_octree.radiusSearch(d_searchQuery, d_radiusSizes, m_minAmountOfRequiredPoints, d_result);

  // copy the result of the radius search from the device back to the host.
  d_result.sizes.download(m_outpuSizesRadiusSearch);
  //result_device.data.download(m_outputDataRadiusSearch);  // download of data not necessary
}


void RadiusOutlierRemoval::FilterCloud(void)
{

  // loop through the whole pointcloud
  for (std::size_t i = 0; i < m_outpuSizesRadiusSearch.size(); ++i)
  {
    // check for unvalid points
    if( (m_outpuSizesRadiusSearch[i] < m_minAmountOfRequiredPoints) || !CheckPointIsValid(m_filteredCloud->at(i)) )
    {
      m_filteredCloud->at(i).r = 255;
      m_filteredCloud->at(i).g = 0;
      m_filteredCloud->at(i).b = 0;

      // copy unvalid points to the noiseCloud if specified by the user
      if(m_copyUnvalidPoints)
      {
        m_noiseCloud->push_back(m_filteredCloud->at(i));
      }

      // set unvalid points to exact 0,0,0 x,y,z coordinates to make them "unvalid"
      m_filteredCloud->at(i).x = 0; //NAN;
      m_filteredCloud->at(i).y = 0; //NAN;
      m_filteredCloud->at(i).z = 0; //NAN; --> leads corrupted data somehow
    }
  }
}


bool RadiusOutlierRemoval::CheckPointIsValid(pcl::PointXYZRGB &point)
{
  // check if point has a nan value
  if(std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
  {
    return 0;
  }

  // check if point has all exact zero coordinates (when data is not filled)
  if( (point.x == 0) || (point.y == 0) || (point.z == 0) )
  {
    return 0;
  }

  // point is valid
  return 1;
}
