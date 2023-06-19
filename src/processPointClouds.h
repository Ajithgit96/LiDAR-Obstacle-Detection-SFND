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
#include "render/box.h"
#include "quiz/cluster/kdtree.h"        //included to make use of Node, kd structs

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    //void proximity(std::vector<bool> &processedPoints,std::vector<int> &cluster,KdTree* tree, const std::vector<std::vector<float>> &points,int index, float distanceTol);

    //std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

    //Declared and defined below functions here, coz when these were defined in processPointClouds.cpp, then I was getting
    //'multiple definition of proximity, first defined here' errors

    void proximity(std::vector<bool> &processedPoints,std::vector<int> &cluster,KdTree* tree, const std::vector<std::vector<float>> &points,int index, float distanceTol){
        processedPoints[index] = true;
        cluster.push_back(index);
        std::vector<int> nearby = tree->search(points[index],distanceTol);
        for(int j=0; j<nearby.size(); j++){
            if(!processedPoints[nearby[j]])
                proximity(processedPoints, cluster, tree, points, nearby[j], distanceTol);
        }
    }

    std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
    {

        // TODO: Fill out this function to return list of indices for each cluster

        std::vector<std::vector<int>> clusters;
        std::vector<bool> processedPoints(points.size(),false); //initiating all elements as false automatically

        for (int index=0; index<points.size(); index++){
            if(processedPoints[index]){
                continue;
            }
            std::vector<int> cluster;
            //proximity is clusterHelper function
            proximity(processedPoints, cluster, tree, points, index, distanceTol);
            clusters.push_back(cluster);
        }
        
        return clusters;

    }
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */