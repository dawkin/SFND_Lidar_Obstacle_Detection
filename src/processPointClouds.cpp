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

    // voxel grid point reduction and region based filtering
  	typename pcl::PointCloud<PointT>::Ptr cloud_region {new pcl::PointCloud<PointT>};
  	typename pcl::PointCloud<PointT>::Ptr cloud_filtered {new pcl::PointCloud<PointT>};
  
  	// apply voxel grid
  	pcl::VoxelGrid<PointT> sor;
  	sor.setInputCloud(cloud);
  	sor.setLeafSize(filterRes, filterRes, filterRes);
  	sor.filter(*cloud_filtered);
  
  	// crop far away point
  	pcl::CropBox<PointT> cropFar(true);
  	cropFar.setInputCloud(cloud_filtered);
  	cropFar.setMin(minPoint);
  	cropFar.setMax(maxPoint);
  	cropFar.filter(*cloud_region);
  	
  	// crop roof point
  	std::vector<int> indices;
  
  	pcl::CropBox<PointT> cropNear(true);
    cropNear.setInputCloud(cloud_region);
    cropNear.setMin(Eigen::Vector4f(-2.5, -1.7, -1, 1));
    cropNear.setMax(Eigen::Vector4f(2.5, 1.7, -.4, 1));
    cropNear.filter(indices);
  
  	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  	for (int point: indices){
      	inliers->indices.push_back(point);
    }
  
  	pcl::ExtractIndices<PointT> extract;
  	extract.setInputCloud(cloud_region);
  	extract.setIndices(inliers);
  	extract.setNegative(true);
  	extract.filter(*cloud_region);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  	// Create two new point clouds, one cloud with obstacles and other with segmented plane
  	typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT> ());
  	typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT> ());
  	// Create extract object
  	pcl::ExtractIndices<PointT> extract;
  	
  	// Extract the inliers -> plane cloud
  	extract.setInputCloud(cloud);
  	extract.setIndices(inliers);
  	extract.setNegative(false);
  	extract.filter(*planeCloud);
  
  	// Extract the other point -> obstacle cloud
  	extract.setNegative(true);
  	extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// find inliers for the cloud.
    // Create Segmentation Object
  
    std::unordered_set<int> inliers = RansacPlane(cloud, maxIterations, distanceThreshold);
	
    if (inliers.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    typename pcl::PointCloud<PointT>::Ptr planeInliers { new typename pcl::PointCloud<PointT> };
	typename pcl::PointCloud<PointT>::Ptr planeOutliers { new typename pcl::PointCloud<PointT> };

	for (int index = 0; index < cloud->points.size(); ++index)
	{
		PointT point = cloud->points[index];
		if (inliers.count(index))
			planeInliers->points.push_back(point);
		else
			planeOutliers->points.push_back(point);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return std::make_pair(planeOutliers, planeInliers);
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	int cloud_size = cloud->points.size();

	// For max iterations
	for (int i = 0; i < maxIterations; i++){ 
		std::unordered_set<int> inliers;

		// Randomly sample subset and fit line
		int index_1 = rand() % cloud_size;
		int index_2 = rand() % cloud_size;
		while(index_1 == index_2 ){
			index_2 = rand() % cloud_size;
		}
      	int index_3 = rand() % cloud_size;
      	while(index_1 == index_3 || index_2 == index_3){
          index_3 = rand() % cloud_size;
        }
		PointT p_1 = cloud->points[index_1];
      	double x1 = p_1.x, y1 = p_1.y, z1 = p_1.z;
		PointT p_2 = cloud->points[index_2];
      	double x2 = p_2.x, y2 = p_2.y, z2 = p_2.z;
      	PointT p_3 = cloud->points[index_3];
      	double x3 = p_3.x, y3 = p_3.y, z3 = p_3.z;
		double A = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
		double B = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
		double C = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
      	double D = -(A*x1 + B*y1 + C*z1);

		for (int index = 0; index < cloud_size; index++){

			PointT point = cloud->points[index];
			double x = point.x;
			double y = point.y;
          	double z = point.z;
			// Measure distance between every point and fitted line
			double d = fabs(A*x + B*y +C*z +D) / sqrt(A*A + B*B + C*C); 
			// If distance is smaller than threshold count it as inlier
			if (d <= distanceTol){ inliers.insert(index); }
		}
		// If new Inliers is bigger than old -> replace
		if (inliers.size() > inliersResult.size()){ inliersResult = inliers; }
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, std::vector<bool>& processed,  const std::vector<std::vector<float>>& points, std::vector<int>& cluster, KdTree* tree, float distanceTol){

	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearby = tree->search(points[indice], distanceTol);

	for (int id: nearby){
		if (!processed[id]){
			clusterHelper(id, processed, points, cluster, tree, distanceTol);
		}
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	for (int index = 0; index < points.size(); index++){
		if (processed[index]){
			continue;
		}
		std::vector<int> cluster;
		clusterHelper(index, processed, points, cluster, tree, distanceTol);
		clusters.push_back(cluster);
	}
 
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
	std::vector<std::vector<float>> points;
	for (auto point: cloud->points){
		points.push_back({point.x, point.y, point.z});
	}
    KdTree* tree (new KdTree);
    for (int  i = 0; i < points.size(); i++){
		tree->insert(points[i], i);
	}
    std::vector<std::vector<int>> cluster_indices = euclideanCluster(points, tree, clusterTolerance);

    for (std::vector<int> cluster: cluster_indices){
		if (cluster.size() <= minSize || cluster.size() >= maxSize){
			// go to next loop if inapropriate cluster size
			continue;
		}
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster {new typename pcl::PointCloud<PointT>};
        for (int idx: cluster){
            cloud_cluster->points.push_back((*cloud)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

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