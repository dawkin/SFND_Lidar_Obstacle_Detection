/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
		pcl::PointXYZ point_1 = cloud->points[index_1];
		pcl::PointXYZ point_2 = cloud->points[index_2];
		double A = point_1.y - point_2.y;
		double B = point_2.x - point_1.x;
		double C = (point_1.x * point_2.y) - (point_2.x - point_1.y);

		for (int index = 0; index < cloud_size; index++){

			pcl::PointXYZ point = cloud->points[index];
			double x = point.x;
			double y = point.y;
			// Measure distance between every point and fitted line
			double d = fabs(A*x + B*y +C) / sqrt(A*A + B*B); 
			// If distance is smaller than threshold count it as inlier
			if (d <= distanceTol){ inliers.insert(index); }
		}
		// If new Inliers is bigger than old -> replace
		if (inliers.size() > inliersResult.size()){ inliersResult = inliers; }
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
		pcl::PointXYZ p_1 = cloud->points[index_1];
      	double x1 = p_1.x, y1 = p_1.y, z1 = p_1.z;
		pcl::PointXYZ p_2 = cloud->points[index_2];
      	double x2 = p_2.x, y2 = p_2.y, z2 = p_2.z;
      	pcl::PointXYZ p_3 = cloud->points[index_3];
      	double x3 = p_3.x, y3 = p_3.y, z3 = p_3.z;
		double A = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
		double B = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
		double C = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
      	double D = -(A*x1 + B*y1 + C*z1);

		for (int index = 0; index < cloud_size; index++){

			pcl::PointXYZ point = cloud->points[index];
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

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
