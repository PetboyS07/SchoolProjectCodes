//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurma
// Date:        15-11-2023
// Project:     Unilogger Software
// Filename:    dataViewer.cpp
// Changed:     03-01-2023 | 14:35
//
// Discription: This file contains all member functions from the dataViewer class.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

// Header Includes
#include "dataViewer.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Function     ////////////////////////////////////////////////////////////////////////////////////////

dataViewer::dataViewer(void)
{
  rgbCloud = nullptr;
  pclViewer = nullptr;

}


dataViewer::~dataViewer(void)
{
  rgbCloud = nullptr;
}


void dataViewer::init(void)
{
    // Open 3D viewer and add point cloud
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Pointcloud Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(rgbCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr emptyPointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pclViewer = viewer;
    pclViewer->setBackgroundColor(0.12, 0.12, 0.12);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(emptyPointCloud, rgb);
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    pclViewer->addCoordinateSystem(0.1);
    pclViewer->initCameraParameters();
    pclViewer->setCameraPosition(0, -0.66, 0.88,    0, 0.35, 0.22,   0, 0.55, 0.84);
    //pclViewer->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    //pclViewer->setCameraClipDistances(0.1,1000);
    cout << "[dataViewer::init] viewer initialized" << endl;
}

bool dataViewer::viewerWasStopped(void)
{
    cout << "[dataViewer::viewerWasStopped] spin viewer" << endl;
    pclViewer->spinOnce(10);
    return pclViewer->wasStopped();
}

void dataViewer::update(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    cout << "[dataViewer::update] update viewer" << endl;
    rgbCloud = cloud;
    pclViewer->updatePointCloud(rgbCloud);
}


void dataViewer::close(void)
{
    
    cout << "[dataViewer::close] close viewer" << endl;
    pclViewer->close();
}


// viewer for a single pcl pointcloud
bool dataViewer::singlePclViewer(char* filepath)
{
    cout << endl;
    cout << "[PCL Viewer] Setting up Viewer.. " << endl;

    // Create pointcloud pointer
    cout << "[Debug] Create pointer" << endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Load pointcloud in pointer
    pcl::io::loadPCDFile(filepath, *cloud);

    // Create Viewer
    shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBViewer(cloud);

    // Set Viewer initial position
    viewer->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    viewer->setCameraClipDistances(0.1,1000);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce (100); // show pointcloud
        usleep(10000);
    }

    // Close the viewer
    viewer->close(); 

    cout << endl;
    cout << "[PCL Viewer] Clossing... " << endl;
    return true;
}


// viewer to compaire unfiltered pointcloud with inliers and outliers.
bool dataViewer::filterCompareViewer(char* filepath)
{
    cout << endl;
    cout << "[PCL Filter Compare] Setting up Filter..." << endl;
    string pointcloudFilePath(filepath);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInliers (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOutliers (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create pointcloud reader
    pcl::PCDReader reader;
    
    // read pointcloud from given file.
    reader.read<pcl::PointXYZRGB> (pointcloudFilePath, *cloud);
    cout << "[PCL Filter Compare] Pointcloud read from filepath." << endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    cout << "[PCL Filter Compare] StatisticalOutliersRemoval filter created." << endl;
    sor.setInputCloud (cloud); // specify the pointcloud to filter
    cout << "[PCL Filter Compare] SOR input cloud set." << endl;
    sor.setMeanK (50); // Number of neighbors to analyse fo each point
    sor.setStddevMulThresh (1.0); // Standard deviation multiplier,
    cout << "[PCL Filter Compare] starting filter..." << endl;
    sor.filter (*cloudInliers); // al point larger than sd of the mean distance to the query point whill be marked as outliers and removed.
    cout << "[PCL Filter Compare] Cloud filterd." << endl;

    // reverse filter to get only the outliers
    sor.setNegative (true);
    cout << "[PCL Filter Compare] starting filter..." << endl;
    sor.filter (*cloudOutliers);
    cout << "[PCL Filter Compare] outlier cloud created." << endl;

    // Create Viewer
    shared_ptr<pcl::visualization::PCLVisualizer> viewerOriginal = createRGBViewer(cloud);
    shared_ptr<pcl::visualization::PCLVisualizer> viewerInliers = createRGBViewer(cloudInliers);
    shared_ptr<pcl::visualization::PCLVisualizer> viewerOutliers = createRGBViewer(cloudOutliers);
    cout << "[PCL Filter Compare] Viewers created." << endl;

    // Set Viewer initial position
    viewerOriginal->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    viewerOriginal->setCameraClipDistances(0.1,1000);
    viewerInliers->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    viewerInliers->setCameraClipDistances(0.1,1000);
    viewerOutliers->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    viewerOutliers->setCameraClipDistances(0.1,1000);
    cout << "[PCL Filter Compare] Viewer settings mate." << endl;

    while (!viewerOriginal->wasStopped() && !viewerInliers->wasStopped() && !viewerOutliers->wasStopped())
    {
        viewerOriginal->spinOnce (100); // show pointcloud
        viewerInliers->spinOnce (100); // show pointcloud
        viewerOutliers->spinOnce (100); // show pointcloud
        usleep(10000);
    }

    // Close the viewer
    viewerOriginal->close();
    viewerInliers->close(); 
    viewerOutliers->close(); 
    cout << "[PCL Filter Compare] Viewers closed" << endl;
    cout << endl;
    cout << "[PCL Viewer Compare] Clossing... " << endl;
    return true;
}

// This function creates a PCL visualizer.
shared_ptr<pcl::visualization::PCLVisualizer> createRGBViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) 
{
    // Open 3D viewer and add point cloud
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Pointcloud Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////