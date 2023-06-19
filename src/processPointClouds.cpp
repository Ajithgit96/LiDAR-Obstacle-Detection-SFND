// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
//#include "quiz/cluster/kdtree.h"        //Included in processPointClouds.h
//#include "quiz/cluster/cluster.cpp"     //This .cpp file also had main function in it - causes error
#include <unordered_set>


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
    //Arguments - Point cloud, Voxel grid size, min/max points representing your region of interest.
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr downsampledCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cropBoxCloud (new pcl::PointCloud<PointT>());

    //Voxel grid downsampling
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*downsampledCloud);

    //Region based filtering
    pcl::CropBox<PointT> cb(true);  //--true specified here gets us the points inside the crop box
    cb.setInputCloud(downsampledCloud);
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.filter(*cropBoxCloud);

    std::vector<int> indices;
    //Removing points from egoCar roof
    pcl::CropBox<PointT> obj(true);
    obj.setInputCloud(cropBoxCloud);
    obj.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    obj.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    obj.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    for(int index : indices)
        inliers->indices.push_back(index);

    //exctracting points other than roof points
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cropBoxCloud);
    extract.setNegative(true);
    extract.setIndices(inliers);
    extract.filter(*cropBoxCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cropBoxCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());

    //for(int index : inliers->indices)
    //    planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planeCloud);
    extract.setNegative (true);
    extract.filter (*obstCloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

/* ----This implementation was with built in functions
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
    */

   //Implementation with RANSAC developed plane segmentation algorithm in ransac2d.cpp
    std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while(maxIterations--){
	
		std::unordered_set<int> inliers;
		while(inliers.size()<3)
			inliers.insert(rand() % cloud->points.size());
		
		//Plane equation
		auto itr = inliers.begin();
		float x1, x2, x3, y1, y2, y3, z1, z2, z3, A, B, C, D;
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		A = (y1-y2)*(z3-z1) - (z2-z1)*(y3-y1);
		B = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		C = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		D = -(A*x1 + B*y1 + C*z1);

		// Measure distance between every point and fitted plane
		for(int index = 0; index < cloud->points.size(); index++)
		{
			if(inliers.count(index)>0)
				continue;
			
			PointT point = cloud->points[index];
			float distance = fabs((A*point.x) + (B*point.y) + (C*point.z) + D) / sqrt(pow(A,2) + pow(B,2) + pow(C,2));
			// If distance is smaller than threshold count it as inlier
			if(distance<=distanceThreshold){
				inliers.insert(index);
			}
		}

		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;
		}
	}

    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
            planeCloud->points.push_back(point);
		else
            obstCloud->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult (obstCloud ,planeCloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Developed plane segmentation alogrithm took " << elapsedTime.count() << " milliseconds" << std::endl; 

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

/*----Clustering implementation with PCL built-in functions
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
*/
/*    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*cloud)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }*/
/*
    //another way of performing above for loop - using range based for loops
    for(pcl::PointIndices getIndices : cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for(int index : getIndices.indices)
            cloud_cluster->points.push_back(cloud->points[index]);
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }
*/

    //----Implementing clustering with custom developed KD Tree and euclidian clustering algorithms
    KdTree* tree = new KdTree;

    //Building a 3D KD Tree
    for(int i=0; i < cloud->points.size(); i++){
        tree->insert({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z},i);
    }

    //Function for eculidian clustering
    std::vector<std::vector<int>> clusters_Indices;
    std::vector<std::vector<float>> points;

    //writing point cloud points into std::vector<std::vector<float>> type
    for(int j=0; j<cloud->points.size(); j++){
        std::vector<float> cloudPoint = {cloud->points[j].x, cloud->points[j].y, cloud->points[j].z};
        points.push_back(cloudPoint);
    }
    clusters_Indices = euclideanCluster(points, tree, clusterTolerance);

    for(std::vector<int> cluster : clusters_Indices)
  	{
        typename pcl::PointCloud<PointT>::Ptr clusterCloud (new pcl::PointCloud<PointT>());
        if(cluster.size()>=minSize && cluster.size()<=maxSize){

/*            for(int indice: cluster)
  			    //clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));  //for pointT = pcl::PointXYZ
                clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));  //for pointT = pcl::PointXYZ
                //clusterCloud->points.push_back(pcl::PointXYZI(points[indice][0],points[indice][1],points[indice][2],0.f));    //for pointT = pcl::PointXYZI - not correct
                */

            //It seems, PointXYZI does not have a constructor like PointXYZI(float _x, float _y, float _z, _intensity)
            //This approach is used as above line of implementation caused errors for the case of PointXYZI
            for (int indice: cluster){
                PointT p;
                p.x = points[indice][0];
                p.y = points[indice][1];
                p.z = points[indice][2];
                clusterCloud->points.push_back(PointT(p));  //--Here PointT(p) to make it compatible for both PointXYZ and PointXYZI
            }
  		    clusters.push_back(clusterCloud);
        }
  	}

  	if(clusters_Indices.size()==0)
  		cout << "No clusters were detected" << endl;

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