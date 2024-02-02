#ifndef UTILITIES_H
#define UTILITIES_H

#include <string>
#include <cmath>
#include <iostream>
#include <chrono>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void calculateStatisticalValues(std::vector<double> values, double &meanValue, double &deviationValue, double &medianValue);
void getFilename(char *argument,  char *& fileName, bool replaceUnderscoreWithWhitespace);
bool CheckPointOutOfRadius(pcl::PointXYZRGB *p1, pcl::PointXYZRGB *p2, float maxRadius);
int getAmountOfValidPointsRow(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool IndexSearchOnWidth0Height1, int rowIndex, int startingIndex, int stoppingIndex);
double calculateRMS(std::vector<double> values);
void setPCLColorRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, uint8_t red, uint8_t blue, uint8_t green);
bool pointIsValid(pcl::PointXYZRGB* point);





#endif // UTILITIES_H
