// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
   

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    typename pcl::PointCloud<PointT>::Ptr cloud_filter(new pcl::PointCloud<PointT>);
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filter);
    typename pcl::PointCloud<PointT>::Ptr cloudregion(new pcl::PointCloud<PointT>);
   
    pcl::CropBox<pcl::PointXYZI> cropbox(true);
    cropbox.setMin(minPoint);
    cropbox.setMax(maxPoint);
    cropbox.setInputCloud(cloud_filter);
    cropbox.filter(*cloudregion);

    std::vector<int> indices;
    pcl::CropBox<pcl::PointXYZI> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudregion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloudregion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudregion);


    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudregion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(/*pcl::PointIndices::Ptr inliers*/std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  	typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr planCloud(new pcl::PointCloud<PointT>);    
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	{
        for (int index = 0; index < cloud->points.size(); index++)
        {
            PointT point = cloud->points[index];
            if (inliers.count(index))
                planCloud->points.push_back(point);
            else
                obstCloud->points.push_back(point);
        }

    }
  
 //USING INBUILD FUNCTIONS----------------------------------------------------------------------- 
  {
  
//     pcl::ExtractIndices<PointT> extract;

//     /*for (size_t i{0} ; i < cloud->size(); i++)*/
//     {
//         extract.setInputCloud(cloud);
//         extract.setIndices(inliers);
//         extract.setNegative(false);
//         extract.filter(*planCloud);
//         extract.setNegative(true);
//         extract.filter(*obstCloud);
//     }


//     std::cout << "Obstacle cloud size: " << obstCloud->size() << std::endl;
//     std::cout << "Plane cloud size: " << planCloud->size() << std::endl;


//     /*std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> planCoud;
//     for (size_t i{ 0 }; i < inliers->indices.size(); i++)
//         planCoud.push_back(inliers->indices);

//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     extract.setNegative(true);
//     extract.filter(*obstCoud);*/
    
  }
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    std::cout << "Cloud size: " << cloud->size() << std::endl;
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

// RANSAC ALGO -----------------------------------------------------------------------------------------------------------------------------------------------------
  
	std::unordered_set<int> inliersResult;
    {
        
        srand(time(NULL));

        // TODO: Fill in this function
        int iterations{ 0 };
        float x1{ 0 }, x2{ 0 }, x3{ 0 }, y1{ 0 }, y2{ 0 }, y3{ 0 }, z1{ 0 }, z2{ 0 }, z3{ 0 };
        int total_points = cloud->size();
        int random_number[3]{};
        size_t count{ 3 };       // number of ramdom number to generate
        int min{ 1 };            // lower bound
        int max{ total_points };      // upper bound
        // For max iterations 

        while (iterations < maxIterations) {
            for (size_t i{ 0 }; i < count; ++i) {
                random_number[i] = (rand() + (i * max)) % max + min;
            }

            //std::cout << std::endl;
            x1 = cloud->points[random_number[0]].x;
            y1 = cloud->points[random_number[0]].y;
            z1 = cloud->points[random_number[0]].z;

            x2 = cloud->points[random_number[1]].x;
            y2 = cloud->points[random_number[1]].y;
            z2 = cloud->points[random_number[1]].z;

            x3 = cloud->points[random_number[2]].x;
            y3 = cloud->points[random_number[2]].y;
            z3 = cloud->points[random_number[2]].z;


            /*std::cout << "Three ramdom numbers: " << random_number[0] << "    " << random_number[1] << "    " << random_number[2] << std::endl;
            std::cout << "x1 y1 z1 point: " << x1 << "   " << y1 << "   " << z1 << std::endl;
            std::cout << "x2 y2 z2 point: " << x2 << "   " << y2 << "   " << z2 << std::endl;
            std::cout << "x3 y3 z3 point: " << x3 << "   " << y3 << "   " << z3 << std::endl;*/

            /*std::cerr << "--------------------------------------------------------------------" << std::endl;*/
            float i{ ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1)) };
            float j{ ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1)) };
            float k{ ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1)) };
            float A{ i }, B{ j }, C{ k };
            float D{ -((i * x1) + (j * y1) + (k * z1)) };

           /* std::cout << "A, B, C and D value: " << A << "   " << B << "   " << C << "   " << D << std::endl;*/

            // Measure distance between every point and fitted line
            float distance{ 0 };
            std::unordered_set<int> inliers{};
            //std::unordered_set<int> max_inliers{};
            for (size_t i{ 0 }; i < total_points; ++i) {
                distance = (fabs((A * (cloud->points[i].x)) + (B * (cloud->points[i].y)) + (C * (cloud->points[i].z)) + D)) / (sqrt((A * A) + (B * B) + (C * C)));
                if (distance < distanceThreshold) {
                    inliers.insert(i);
                }
                /*std::cout << distance << "   " << inliers.size() << std::endl;*/
            }
            /*std::cout << inliers.size() << std::endl;
            std::cout << max_inliers.size() << std::endl;*/
            if (inliers.size() > inliersResult.size()) {
                inliersResult = inliers;
            }
          	++iterations;
            /*std::cout << "\nThe inlier vector elements are : ";
            for (int i = 0; i < inliers.size(); i++)
                std::cout << inliers.at(i) << ' ';
            std::cout << "\nThe max inlier vector elements are : ";
            for (int i = 0; i < max_inliers.size(); i++)
                std::cout << max_inliers.at(i) << ' ';*/
            /*for (int x : max_inliers) {
                inliersResult.insert(x);
            }*/
            //inliersResult.clear();
            /*inliersResult(max_inliers.begin(), max_inliers.end());*/
            //std::copy(max_inliers.begin(), max_inliers.end(), std::inserter(inliersResult, inliersResult.end()));
            /*for (const int& i : max_inliers) {
                inliersResult.insert(i);
            }*/
            /*std::copy(max_inliers.begin(), max_inliers.end(), std::inserter(inliersResult, inliersResult.end()));*/
            
            /*std::cout << "\nThe max inlierResult elements are : ";
            for (auto const& i : inliersResult) {
                std::cout << i << " ";
            }*/
        }

    }
  
   

  
// Segmentation using INBUILD FUNCTION (All commented)------------------------------------------------------------------------------------------------------
  
 	{
  
      //   	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      //     // TODO:: Fill in this function to find inliers for the cloud.  
      //     pcl::SACSegmentation<PointT> seg;
      //     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
      //     seg.setOptimizeCoefficients(true);
      //     seg.setModelType(pcl::SACMODEL_PLANE);
      //     seg.setMethodType(pcl::SAC_RANSAC);
      //     seg.setMaxIterations(maxIterations);
      //     seg.setDistanceThreshold(distanceThreshold);

      //     seg.setInputCloud(cloud);
      //     seg.segment(*inliers, *coefficients);
      //     if (inliers->indices.size() == 0)
      //     {
      //         std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;

      //     }
      //     std::cout << "Inliers size: " << inliers->indices.size() << std::endl;
 	}
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
// Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>);
    (*tree).setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);


    
    //cluster->push_back(cloud);
   // int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cloud_cluster->push_back((*cloud)[*pit]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        std::cout << "Cloud Cluster size: " << cloud_cluster->size() << std::endl;
        clusters.push_back(cloud_cluster);
        //j++;
    }
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}


//Euclidean Clustering -----------------------------------------------------------------------------------------------------------------------------------------




template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, int& id, KdTree* tree, float distanceTol, std::vector<bool> &processed) {
  
    processed[id] = true;
    cluster.push_back(id);
    std::vector<float> point;
    point = { cloud->points[id].x, cloud->points[id].y, cloud->points[id].z };
    std::vector<int> nearby = tree->search(point, distanceTol);
    for (int index : nearby) {
        if (!processed[index]) {
            Proximity(cloud, cluster, index, tree, distanceTol, processed);
        }
    }
}


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{
    
    auto startTime = std::chrono::steady_clock::now();
    KdTree* tree = new KdTree;
    for (int i = 0; i < cloud->size(); i++)
        tree->KdTree::insert({ cloud->points[i].x, cloud->points[i].y, cloud->points[i].z }, i);

    std::vector<bool> processed(cloud->size(), false);

    std::vector<std::vector<int>> clusters;

    for (int i{ 0 }; i < cloud->size(); i++) {
		if (processed[i]) {
			++i;
			continue;
		}
		std::vector<int> cluster;
		Proximity(cloud, cluster, i, tree, distanceTol, processed);
		if (cluster.size() > minSize && cluster.size() < maxSize) {
			clusters.push_back(cluster);
		}

		
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "EuclideanClustering took" << elapsedTime.count() << " milliseconds" << std::endl;

    return clusters;
}