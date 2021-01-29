/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    // TODO:: Create lidar sensor
    Lidar* lidar_sensor_ptr = new Lidar(cars, 0);
  	//pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_lidar = lidar_sensor_ptr->scan();
    //renderRays(viewer, (*lidar_sensor_ptr).position, pointcloud_lidar);               //renderRays(viewer, { 0, 0, 2.3 }, pointcloud_lidar);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcd = lidar_sensor_ptr->scan();
    //renderPointCloud(viewer, pointcloud_pcd, "PCD");
     

    // TODO:: Create point processormake
  	ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    /* ProcessPointClouds<pcl::PointXYZ> pointProcessor;*/
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(pointcloud_pcd, 25, 0.3);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
  
  	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudcluster = pointProcessor->Clustering(segmentCloud.first, 0.49, 10, 500);
    int clusterID = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(1,1,0), Color(0,0,1) };

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudcluster) {
        std::cout << "Cluster Size: ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterID), colors[clusterID]);
      	Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterID);
        ++clusterID;
    }

  
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  	//Filtering-------------------------------------------------------------------------------------------------------

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3f , Eigen::Vector4f(-10.12, -5.12 , -2.12, 1), Eigen::Vector4f(30, 7.12, 10.12, 1));
    //renderPointCloud(viewer, filterCloud, "filterCloud");

    //RANSAC - segmentaion------------------------------------------------------------------------------------------------

    ////////ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 25, 0.15);
    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));  
    
    //EuclideanClustering ----------------------------------------------------------------------------------------------------

    std::vector<std::vector<int>> clusters = pointProcessorI->euclideanCluster(segmentCloud.first, 0.55, 10, 1500);


    int clusterID{ 0 };
    std::vector<Color> colors = { Color(1,0,1), Color(1,1,0), Color(0,0,1) };
    for (std::vector<int> cluster : clusters) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud < pcl::PointXYZI>());
        for (int indice : cluster) {
            clusterCloud->points.push_back(segmentCloud.first->points[indice]);
        }
      	std::cout << "Cluster Size: " << clusterCloud->size() << std::endl;        
        renderPointCloud(viewer, clusterCloud, "Cluster" + std::to_string(clusterID), colors[clusterID % colors.size()]);
        Box box = pointProcessorI->BoundingBox(clusterCloud);
        renderBox(viewer, box, clusterID);
        ++clusterID;
    }


  
//USING In build Functions----------------------------------------------------------------------------------------------------------------
  
  {
  
  
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    ///////ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    /////pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer, inputCloud, "inputCloud");


    //filtering
    //pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3f , Eigen::Vector4f(-10.12, -5.12 , -2.12, 1), Eigen::Vector4f(30.12, 7.12, 10.12, 1));
    //renderPointCloud(viewer, filterCloud, "filterCloud");

    //segmentaion
    ////////ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 25,0.3);
    ///////renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    ///renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));  

    //Clustring
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudcluster = pointProcessorI->Clustering(segmentCloud.first, 0.53, 10, 500);
    //int clusterID = 0;
    //std::vector<Color> colors = { Color(1,0,1), Color(1,1,0), Color(0,0,1), Color(1,0,0), Color(1,1,1), Color(1,0,1) };



//     for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudcluster) {
//        std::cout << "Cluster Size: ";
//        pointProcessorI->numPoints(cluster);
//         renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterID), colors[clusterID%colors.size()]);
//        Box box = pointProcessorI->BoundingBox(cluster);
//         renderBox(viewer, box, clusterID);
//         ++clusterID;
//     }
  }

}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
  
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    

    while (!viewer->wasStopped ())
    {
      	// Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();
      
        viewer->spinOnce ();
    } 
}