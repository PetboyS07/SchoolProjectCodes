//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurma
// Date:        15-11-2023
// Project:     Unilogger Software
// Filename:    dataProcessor.cpp
// Changed:     18-01-2023 | 10:33
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
  m_transformedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  filteredCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  m_croppedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
}


// class destructor
dataProcessor::~dataProcessor(void)
{
  Log() << "[dataProcessor::~dataProcessor] Start desctructor\n";
  insertedCloud = nullptr;
  filteredCloud = nullptr;
  m_transformedCloud = nullptr;  
  m_croppedCloud = nullptr;
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
  projectPlane();
  cropPointCloud(3.9, 1.8, 0.7); // crop the pointcloud to a max width, max distance, and overall max depth [meter] beste waarde: (3.9, 1.8, 0.7)
  Log() << "[dataProcessor::processCloud] Processes finisched.\n";
}


// Function to recieve the processed cloud.
void dataProcessor::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputCloud)
{
  if(m_croppedCloud == nullptr)
  {
    Log() << "[Processing]: there is no cloud to get\n";
    return;
  }

  // pass through the pointer address.
  outputCloud = m_croppedCloud;
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
}


void dataProcessor::projectPlane(void)
{
  planeController transformPlane;

  transformPlane.SetInputCloud(filteredCloud);
  transformPlane.SetAveragingIndexWidth(30);
  transformPlane.SetMinPointsValidLine(100);
  transformPlane.Transform(m_transformedCloud);
}


void dataProcessor::cropPointCloud(float width, float distance, float depth)
{
  // build the conditions
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr conditionalCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GT, -(width / 2.0f))));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, (width / 2.0f))));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, distance)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::GT, 0.0)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, -(depth / 2.0f))));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, (depth / 2.0f))));

  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (m_transformedCloud);
  condrem.setKeepOrganized(true);

  // apply filter
  condrem.filter (*conditionalCloud); //  conditionalCloud

  // reindex the pointcloud
  int firstValidXIndex = -1;
  int firstValidYIndex = -1;
  int lastValidXIndex = -1;
  int lastValidYIndex = -1;

  for( int y = 0; y < conditionalCloud->height; y++)
  {

    for( int x = 0; x < conditionalCloud->width; x++)
    {
      if(pointIsValid(&conditionalCloud->at(x, y)))
      {
        if((x < firstValidXIndex) || (firstValidXIndex < 0))
        {
          firstValidXIndex = x;
        }
        if((x > lastValidXIndex) || (lastValidXIndex < 0))
        {
          lastValidXIndex = x;
        }

        if((y < firstValidYIndex) || (firstValidYIndex < 0))
        {
          firstValidYIndex = y;
        }
        if((y > lastValidYIndex) || (lastValidYIndex < 0))
        {
          lastValidYIndex = y;
        }

      }
    }
  }
  int minValidWidthPoints = 150;
  int minValidHeightPoints = 150;
  bool enoughWidth1 = false;
  bool enoughWidth2 = false;
  bool enoughHeight1 = false;
  bool enoughHeight2 = false;

  // adjust the min/max index places in horizontal/vertial directions if needed
  while(!enoughWidth1 || !enoughWidth2 || !enoughHeight1 || !enoughHeight2)
  {

    // reshaping with the provided amount of min valid values is just not possible! --> lower your min. required valid points value's
    if((firstValidXIndex >= (conditionalCloud->width - 1)) || (firstValidYIndex > (conditionalCloud->height - 1)) || (lastValidXIndex <= 0) || (lastValidYIndex <= 0))
    {
      std::cerr << "[reindex pcl]: It is not possible to form a new indexed pointcloud with the given amount of min. valid points for a line, adjust your values!" << std::endl;
      firstValidXIndex = firstValidYIndex = 0;
      lastValidXIndex = conditionalCloud->width - 1;
      lastValidYIndex = conditionalCloud->height - 1;
      return;
    }

    if(!enoughWidth1)
    {
      firstValidYIndex++;
      enoughWidth1 = getAmountOfValidPointsRow(conditionalCloud, 0, firstValidYIndex, firstValidXIndex, lastValidXIndex) >= minValidWidthPoints;
    }

    if(!enoughWidth2)
    {
      lastValidYIndex--;
      enoughWidth2 = getAmountOfValidPointsRow(conditionalCloud, 0, lastValidYIndex, firstValidXIndex, lastValidXIndex) >= minValidWidthPoints;
    }

    if(!enoughHeight1)
    {
      firstValidXIndex++;
      enoughHeight1 = getAmountOfValidPointsRow(conditionalCloud, 1, firstValidXIndex, firstValidYIndex, lastValidYIndex) >= minValidHeightPoints;
    }

    if(!enoughHeight2)
    {
      lastValidXIndex--;
      enoughHeight2 = getAmountOfValidPointsRow(conditionalCloud, 1, lastValidXIndex, firstValidYIndex, lastValidYIndex) >= minValidHeightPoints;
    }

  }

  m_croppedCloud->width = (lastValidXIndex + 1) - firstValidXIndex;
  m_croppedCloud->height =(lastValidYIndex + 1) - firstValidYIndex;
  m_croppedCloud->is_dense = false;
  m_croppedCloud->points.resize (m_croppedCloud->width * m_croppedCloud->height);
  for(int x = 0; x < m_croppedCloud->width; x++)
  {
    for(int y = 0; y < m_croppedCloud->height; y++)
    {
      pcl::copyPoint(conditionalCloud->at((firstValidXIndex + x), (firstValidYIndex + y)), m_croppedCloud->at(x, y));
    }

  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////