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

#include "planeController.hpp"


planeController::planeController(void)
{
  planeHeightZ = 1.5; // beste afmeting: 1.5
  
  mBar1Data.averageX = mBar1Data.averageY = mBar1Data.averageZ = 0;
  mBar1Data.barLengths.clear();
  mBar1Data.validBarPoints.clear();
  mBar1Data.medianBarLength = 0;

  mBar2Data.averageX = mBar2Data.averageY = mBar2Data.averageZ = 0;
  mBar2Data.barLengths.clear();
  mBar2Data.validBarPoints.clear();
  mBar2Data.medianBarLength = 0;

  inCloud = nullptr;
  transformationCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  outCloud = nullptr;
  mMinPointsValidLine = 0;
  mIndexSearchWidth = 0;
  mPlaneAngle = 0;
  mPlaneTransl.x = mPlaneTransl.y = mPlaneTransl.z = 0;
  mNeedFlipping = false;
}


void planeController::SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud)
{
  // check if the input cloud is 2D indexed
  if (!inputCloud->isOrganized())
  {
    std::cout << "[plane transformation]: Cloud does not have an organized pointcloud, exiting\n";
    return;
  }
  inCloud = inputCloud;
}


void planeController::SetAveragingIndexWidth(int width)
{
  // min 1 index run have to be done.
  if(width >= 1)
    mIndexSearchWidth = width;
  else
    mIndexSearchWidth = 1;
}


void planeController::SetMinPointsValidLine(int nrOfPoints)
{
  // at least the two outer points have to be valid.
  if(nrOfPoints >= 2)
    mMinPointsValidLine = nrOfPoints;
  else
    mMinPointsValidLine = 2;
}


void planeController::Transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud)
{
  // check for unvalid input parameters
  if(( mMinPointsValidLine == 0 ) || ( mIndexSearchWidth == 0 ) || ( inCloud == nullptr ))
  {
    std::cout << "[Plane Transformation]: Unvalid input parameter(s) were specified!" << std::endl;
    return;
  }

  // check for unvalid output cloud
  if(outputCloud == nullptr)
  {
    std::cout << "[Plane Transformation]: No valid output cloud was specified!" << std::endl;
    return;
  }

  outCloud = outputCloud;

  transformationCloud->width = inCloud->width;
  transformationCloud->height = inCloud->height;
  transformationCloud->is_dense = inCloud->is_dense;
  transformationCloud->points.resize(inCloud->width * inCloud->height);
  pcl::copyPointCloud(*inCloud, *transformationCloud);

  // Calculate average point inner/outer bar data.
  if(!CalculateBar1Point())
  {
    return;
  }
  if(!CalculateBar2Point())
  {
    return;
  }

  // Calculate plane angle and translation compared to camera origin.
  CalculatePlaneAngle();
  CalculatePlaneTranslation();

  // Transform the plane to zero angle and camera origin in the center of the inner bar.
  TransformPlaneToMid();

}

bool planeController::CalculateBar1Point(void)
{


  // set index parameters to -1 as default value
  int p1, p2; // = -1, p2 = -1;//, lineIndex = -1;
  //int validPointsTotal;
  int unvalidStartLines = 0;
  int validLinePoints;

  // loop through the index-bottom lines
  for(int y = 0; y < mIndexSearchWidth + unvalidStartLines; y++)
  {
    p1 = p2 = -1;
    validLinePoints = 0;

    // check out of range
    if(y >= transformationCloud->height)
    {
      break;
    }

    // count the number of valid points for the line
    for(int x = 0; x < transformationCloud->width; x++)
    {

      // check if the point is not valid
      if(!CheckPointIsValid(&transformationCloud->at(x, y)))
      {
        continue;
      }

      // check if the first valid point is set.
      if(p1 == -1)
      {
        p1 = x;
      }
      // set the last valid point to p2
      p2 = x;

      // push the valid line points to the vector
      mBar1Data.validBarPoints.push_back(transformationCloud->at(x,y));
      validLinePoints++;
    }

    // check the amount of valid points for the line.
    if(validLinePoints < mMinPointsValidLine)
    {
      // pop the points of the unvalid line out of the vector
      for(int k = 0; k < validLinePoints; k++)
      {
        mBar1Data.validBarPoints.pop_back();
      }

      unvalidStartLines++;
      continue;
    }

    // calculate the bar the valid bar length and push to the vector
    double squaredDifferenceX = std::pow((transformationCloud->at(p2, y).x - transformationCloud->at(p1, y).x), 2);
    double squaredDifferenceY = std::pow((transformationCloud->at(p2, y).y - transformationCloud->at(p1, y).y), 2);
    double squaredDifferenceZ = std::pow((transformationCloud->at(p2, y).z - transformationCloud->at(p1, y).z), 2);
    mBar1Data.barLengths.push_back( std::sqrt( squaredDifferenceX + squaredDifferenceY + squaredDifferenceZ ) );

  }

  // check if not enough bars are calculated (Out of Range errror)
  if(mBar1Data.barLengths.size() < mIndexSearchWidth)
  {
    std::cout << "Bar 1 cannot be established!" << "\n";
    return false;
  }

  // Calculate the median bar length and average bar coordinate
  mBar1Data.medianBarLength = CalculateMedianValue(mBar1Data.barLengths, mBar1Data.barLengths.size());
  CalculateAverageCoordinates(mBar1Data);

  return true;
}

bool planeController::CalculateBar2Point(void)
{
    // set index parameters to -1 as default value
    int p1, p2; // = -1, p2 = -1;//, lineIndex = -1;
    int unvalidStartLines = 0;
    int validLinePoints;

    // loop through the index-top lines
    for(int y = (transformationCloud->height - 1); y >= ((transformationCloud->height - 1) - mIndexSearchWidth - unvalidStartLines); y--)
    {
      p1 = p2 = -1;
      validLinePoints = 0;

      // check out of range
      if(y < 0)
      {
        break;
      }

      // count the number of valid points for the line
      for(int x = 0; x < transformationCloud->width; x++)
      {

        // check if the point is not valid
        if(!CheckPointIsValid(&transformationCloud->at(x, y)))
        {
          continue;
        }

        // check if the first valid point is set.
        if(p1 == -1)
        {
          p1 = x;
        }
        // set the last valid point to p2
        p2 = x;

        // push the valid line points to the vector
        mBar2Data.validBarPoints.push_back(transformationCloud->at(x,y));
        validLinePoints++;
      }

      // check the amount of valid points for the line.
      if(validLinePoints < mMinPointsValidLine)
      {
        // pop the points of the unvalid line out of the vector
        for(int k = 0; k < validLinePoints; k++)
        {
          mBar2Data.validBarPoints.pop_back();
        }

        unvalidStartLines++;
        continue;
      }

      // calculate the bar the valid bar length and push to the vector
      double squaredDifferenceX = std::pow((transformationCloud->at(p2, y).x - transformationCloud->at(p1, y).x), 2);
      double squaredDifferenceY = std::pow((transformationCloud->at(p2, y).y - transformationCloud->at(p1, y).y), 2);
      double squaredDifferenceZ = std::pow((transformationCloud->at(p2, y).z - transformationCloud->at(p1, y).z), 2);
      mBar2Data.barLengths.push_back( std::sqrt( squaredDifferenceX + squaredDifferenceY + squaredDifferenceZ ) );

    }

    // check if not enough bars are calculated (Out of Range errror)
    if(mBar2Data.barLengths.size() < mIndexSearchWidth)
    {
      std::cout << "Bar 2 cannot be established!" << "\n";
      return false;
    }

    // Calculate the median bar length and average bar coordinate
    mBar2Data.medianBarLength = CalculateMedianValue(mBar2Data.barLengths, mBar2Data.barLengths.size());
    CalculateAverageCoordinates(mBar2Data);

    return true;
}


void planeController::CalculatePlaneAngle(void)
{
  float deltaHeight;
  float deltaLength;


  deltaHeight = std::abs(mBar1Data.averageZ - mBar2Data.averageZ);
  deltaLength = std::abs(mBar1Data.averageY - mBar2Data.averageY);

  mPlaneAngle = std::atan(deltaHeight/deltaLength);

  // get the mbardata of the outer bar and determine direction of the angle
  if(std::abs(mBar1Data.medianBarLength) > std::abs(mBar2Data.medianBarLength))
  {
    if(mBar1Data.averageY < 0)
    {
      mPlaneAngle = -mPlaneAngle;
    }
  }
  else
  {
    if(mBar2Data.averageY < 0)
    {
       mPlaneAngle = -mPlaneAngle;
    }
  }
}


void planeController::CalculatePlaneTranslation(void)
{
  if(std::abs(mBar1Data.medianBarLength) < std::abs(mBar2Data.medianBarLength))
  {
    mPlaneTransl.x = -mBar1Data.averageX;
    mPlaneTransl.y = -mBar1Data.averageY;
    mPlaneTransl.z = -mBar1Data.averageZ;
    if(mBar2Data.averageY < 0)
    {
      mNeedFlipping = true;
    }
    else
    {
      mNeedFlipping = false;
    }
  }
  else
  {
    mPlaneTransl.x = -mBar2Data.averageX;
    mPlaneTransl.y = -mBar2Data.averageY;
    mPlaneTransl.z = -mBar2Data.averageZ;
    if(mBar1Data.averageY < 0)
    {
      mNeedFlipping = true;
    }
    else
    {
      mNeedFlipping = false;
    }

  }
}


void planeController::TransformPlaneToMid(void)
{

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr translationCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotationCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr flippedCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> unvalidIndices;

    // store all unvalid points in vector, these points have to stay zero after the transformation
    for(int i = 0; i < inCloud->size(); i++)
    {
      if(!CheckPointIsValid(&inCloud->at(i)))
        unvalidIndices.push_back(i);
    }

    // setup translation part of the matrice
    transform(0,3) = mPlaneTransl.x;
    transform(1,3) = mPlaneTransl.y;
    transform(2,3) = planeHeightZ;//1.5;
    transform(0,0) = 1;
    transform(1,1) = 1;
    transform(2,2) = 1;
    transform(3,3) = 1;

    pcl::transformPointCloud (*transformationCloud, *translationCloud, transform);


    transform.setConstant(4, 4, 0);
    // setup rotation part of the matrice
    transform(1,1) = std::cos(mPlaneAngle);
    transform(1,2) = -std::sin(mPlaneAngle);
    transform(2,1) = std::sin(mPlaneAngle);
    transform(2,2) = std::cos(mPlaneAngle);
    transform(0,0) = 1;
    transform(3,3) = 1;
    pcl::transformPointCloud (*translationCloud, *rotationCloud, transform);

    if(mNeedFlipping)
    {
       transform.setConstant(4, 4, 0);
       // setup 180 degrees flip part of the matrice
       transform(0,0) = std::cos(M_PI);
       transform(0,1) = -std::sin(M_PI);
       transform(1,0) = std::sin(M_PI);
       transform(1,1) = std::cos(M_PI);
       transform(2,2) = 1;
       transform(3,3) = 1;
       pcl::transformPointCloud (*rotationCloud, *flippedCloud, transform);
    }
    else
    {
      // set the flipped cloud pointer directly to the rotationcloud if the flip 180 degrees is not needed
      flippedCloud = rotationCloud;
    }



    // restore all unvalid points to zero
    for(int j = 0; j < unvalidIndices.size(); j++)
    {
      flippedCloud->at(unvalidIndices[j]).x = 0; // flippedCloud
      flippedCloud->at(unvalidIndices[j]).y = 0;
      flippedCloud->at(unvalidIndices[j]).z = 0;
      flippedCloud->at(unvalidIndices[j]).rgb = 0; //flippedCloud
    }

    // copy the transformed plane cloud to the output cloud
    pcl::copyPointCloud(*flippedCloud, *outCloud); //flippedCloud
}

double planeController::CalculateMedianValue(std::vector<double> vect, int size)
{

    for (auto median : vect)
    {
      //std::cout << "median value: " << median << std::endl;
    }

  // If size of the arr[] is even
  if (size % 2 == 0)
  {
    // Applying nth_element
    // on n/2th index
    std::nth_element(vect.begin(),
                vect.begin() + size / 2,
                vect.end());

    // Applying nth_element
    // on (n-1)/2 th index
    std::nth_element(vect.begin(),
                vect.begin() + (size - 1) / 2,
                vect.end());

    // Find the average of value at
    // index N/2 and (N-1)/2
    return (double)( vect[(size - 1) / 2] + vect[size / 2] ) / 2.0;
  }

  // If size of the arr[] is odd
  else
  {
    // Applying nth_element
    // on n/2
    std::nth_element(vect.begin(),
                vect.begin() + size / 2,
                vect.end());

    // Value at index (N/2)th
    // is the median
    return (double)vect[size / 2];
  }

}

void planeController::CalculateAverageCoordinates(barData &bar)
{
  bar.averageX = bar.averageY = bar.averageZ = 0;

  for (auto point : bar.validBarPoints)
  {
    bar.averageX += point.x;
    bar.averageY += point.y;
    bar.averageZ += point.z;
  }

  bar.averageX /= bar.validBarPoints.size();
  bar.averageY /= bar.validBarPoints.size();
  bar.averageZ /= bar.validBarPoints.size();

}

bool planeController::CheckPointIsValid(pcl::PointXYZRGB* point)
{
  // check if point has a nan value
  if(std::isnan(point->x) || std::isnan(point->y) || std::isnan(point->z))
    return 0;

  // check if point has any zero coordinates (when data is not filled)
  if( (point->x == 0) || (point->y == 0) || (point->z == 0) )
    return 0;

  // point is valid
  return 1;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     End ;)     //////////////////////////////////////////////////////////////////////////////////////////