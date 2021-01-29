// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"

struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
		: root(NULL)
	{}

	//3D tree
	void node_creator(std::vector<float> point, int id, Node** node_address, unsigned int depth) {
		if (*node_address == NULL) {
			*node_address = new Node(point, id);

		}
		else {
			unsigned int axis = depth % 3;
			if (point[axis] < (*node_address)->point[axis]) {
				node_creator(point, id, &((*node_address)->left), depth + 1);
			}
			else {
				node_creator(point, id, &((*node_address)->right), depth + 1);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		
		node_creator(point, id, &root, 0);

	}

	//for 3D tree
	void search_helper(std::vector<float> target, float distanceTol, Node* node_address, int depth, std::vector<int>& ids) {

		if (node_address != NULL) {
			
			if ((node_address->point[0] <= (target[0] + distanceTol)) && (node_address->point[0] >= (target[0] - distanceTol)) && (  node_address->point[1] >= (target[1] - distanceTol)) && (node_address->point[1] <= (target[1] + distanceTol)) &&  (node_address->point[2] >= (target[2] - distanceTol)) && (node_address->point[2] <= (target[2] + distanceTol))) {

				float distance = sqrt(((node_address->point[0] - target[0]) * (node_address->point[0] - target[0])) + ((node_address->point[1] - target[1]) * (node_address->point[1] - target[1]))
				+ ((node_address->point[2] - target[2]) * (node_address->point[2] - target[2])));
				/*std::cout << distance << std::endl;*/
				if (distance <= distanceTol) {
					ids.push_back(node_address->id);
					/*std::cout << ids[0] << std::endl;*/
				}

			}

			if ((node_address->point[depth % 3]) > (target[depth % 3] - distanceTol)) {
				search_helper(target, distanceTol, node_address->left, depth +1, ids);
			}
			if ((node_address->point[depth % 3]) < (target[depth % 3] + distanceTol)) {
				search_helper(target, distanceTol, node_address->right, depth +1, ids);
			}
		}
	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(target, distanceTol, root, 0, ids);
		return ids;
	}

};


template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(/*pcl::PointIndices::Ptr inliers*/std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    
    void Proximity(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, int& id, KdTree* tree, float distanceTol, std::vector<bool> &processed);

	std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize);
  
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */