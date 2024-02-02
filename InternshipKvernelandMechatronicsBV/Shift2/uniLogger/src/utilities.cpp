#include "utilities.hpp"


void getFilename(char *argument, char *& fileName, bool replaceUnderscoreWithWhitespace)
{
  const int argStrSize = strlen(argument);
  int fileCharlength = 0;
  for(int n = (argStrSize - 1); n >= 0; n--)
  {
    if(argument[n] == '/')
    {
      break;
    }
    else
    {
      fileCharlength++;
    }
  }

  fileName = new char[fileCharlength + 1]; // include the terminator char

  for(int n = 0; n < (fileCharlength); n++)
  {
    if((argument[argStrSize - fileCharlength + n] == '_' ) && replaceUnderscoreWithWhitespace)
    {
      fileName[n] = ' ';
      continue;
    }

    fileName[n] = argument[argStrSize - fileCharlength + n];
  }
  // set the terminator char
  fileName[fileCharlength] = 0;
}

int getAmountOfValidPointsRow(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool IndexSearchOnWidth0Height1, int rowIndex, int startingIndex, int stoppingIndex)
{
  int iterator = startingIndex;

  // reference vars to check the amount of valid points of the according line
  int& xIndex = (IndexSearchOnWidth0Height1) ? rowIndex : iterator;
  int& yIndex = (IndexSearchOnWidth0Height1) ? iterator : rowIndex;

  int amountOfValidPoints = 0;
  while(iterator <= stoppingIndex)
  {
    if(pointIsValid(&cloud->at(xIndex, yIndex)))
    {
      amountOfValidPoints++;
    }
    iterator++;

  }
  return amountOfValidPoints;
}

void calculateStatisticalValues(std::vector<double> values, double &meanValue, double &deviationValue, double &medianValue)
{
  // calculate the mean value of the data
  meanValue = 0;
  for(auto value : values)
  {
    meanValue += static_cast<double>(value);
  }
  meanValue /= values.size();


  // calculate the squared deviation for the standard deviation of the distribution
  double squaredDeviationValuesSum = 0;
  for(auto value : values)
  {
    double difference = value - meanValue;
    squaredDeviationValuesSum += (difference * difference);
  }

  // calculate the standard deviation value
  double meanSquaredDeviation = squaredDeviationValuesSum / ( values.size() - 1 );
  deviationValue = std::sqrt( meanSquaredDeviation );


  // If size of the arr[] is even
  if (values.size() % 2 == 0)
  {
    // Applying nth_element
    // on n/2th index
    std::nth_element(values.begin(),
                values.begin() + values.size() / 2,
                values.end());

    // Applying nth_element
    // on (n-1)/2 th index
    std::nth_element(values.begin(),
                values.begin() + (values.size() - 1) / 2,
                values.end());

    // Find the average of value at
    // index N/2 and (N-1)/2
    medianValue = (double)( values[(values.size() - 1) / 2] + values[values.size() / 2] ) / 2.0;
  }

  // If size of the arr[] is odd
  else
  {
    // Applying nth_element
    // on n/2
    std::nth_element(values.begin(),
                values.begin() + values.size() / 2,
                values.end());

    // Value at index (N/2)th
    // is the median
    medianValue = (double)values[values.size() / 2];
  }

}

double calculateRMS(std::vector<double> values)
{
  double sumSquaredValues = 0.0f;
  double RMSValue;

  for(auto p : values)
  {
    sumSquaredValues += ( p * p );
  }

  RMSValue = std::sqrt( sumSquaredValues / values.size() );

  return RMSValue;
}

bool pointIsValid(pcl::PointXYZRGB* point)
{
  // check if point has a nan value
  if(std::isnan(point->x) || std::isnan(point->y) || std::isnan(point->z))
    return 0;

  // check if point has all zero coordinates (when data is not filled)
  if( (point->x == 0) || (point->y == 0) || (point->z == 0) )
    return 0;

  // point is valid
  return 1;
}

void setPCLColorRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, uint8_t red, uint8_t blue, uint8_t green)
{
  for(auto &p: cloud->points)
    {
      p.r = red;
      p.g = blue;
      p.b = green;
  }
}

bool CheckPointOutOfRadius(pcl::PointXYZRGB *p1, pcl::PointXYZRGB *p2, float maxRadius)
{
  // checks in all 3 dimensions if the point lies in the radius
  if(std::abs(p2->x - p1->x) > maxRadius)
    return 1;
  if(std::abs(p2->y - p1->y) > maxRadius)
    return 1;
  if(std::abs(p2->y - p1->y) > maxRadius)
    return 1;

  // point lies in the radius
  return 0;
}
