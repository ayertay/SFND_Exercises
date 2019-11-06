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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());
    for (int index : inliers->indices)
    {
      planeCloud->points.push_back(cloud->points[index]);
    }
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
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
    /* //pcl verion:

    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    // TODO:: Fill in the function to segment cloud into two parts, the drivable plane and obstacles

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment (*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset" << std::endl;
    }
    */
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    srand(time(NULL));
    int max_inliers = 0;
    int max_A = 0;
    int max_B = 0;
    int max_C = 0;
    int max_D = 0;
    // TODO: Fill in this function
    if (cloud->points.size () > 3)
    {

    // For max iterations
        for (int index = 0; index < maxIterations; index++)
        {
        // Randomly sample subset and fit line
        int num_inliers = 0;
        std::unordered_set<int> inlier_set;
        while (inlier_set.size() < 3)
            inlier_set.insert(rand() % cloud->points.size());

        float x1, x2, x3, y1, y2, y3, z1, z2, z3;

        auto itr = inlier_set.begin();
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

        float i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
        float j = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
        float k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
        float A = i;
        float B = j;
        float C = k;
        float D = -(i*x1 + j*y1 + k*z1);
        for (int ctr = 0; ctr < cloud->points.size (); ctr++)
        {
            float x = cloud->points[ctr].x;
            float y = cloud->points[ctr].y;
            float z = cloud->points[ctr].z;
            // Measure distance between every point and fitted line
            float d = fabs(A*x + B*y + C*z + D)/sqrt(pow(A,2) + pow(B,2) + pow(C,2));
            // If distance is smaller than threshold count it as inlier
            if (d <= distanceThreshold)
            {
                num_inliers++;
            }
        
        }
        if (num_inliers > max_inliers)
        {
            max_inliers = num_inliers; 
            max_A = A;
            max_B = B;
            max_C = C;
            max_D = D;
        }
      
        }
        for (int ctr = 0; ctr < cloud->points.size (); ctr++)
        {
            // Measure distance between every point and fitted line
            float d = fabs(max_A*cloud->points[ctr].x + max_B*cloud->points[ctr].y + max_C*cloud->points[ctr].z + max_D)/sqrt(pow(max_A,2) + pow(max_B,2) + pow(max_C,2));
            // If distance is smaller than threshold count it as inlier
            if (d <= distanceThreshold)
            {
                inliers->indices.push_back(ctr);
            }
        }
    }

    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    //own version
    /* MyKdTree* tree = new MyKdTree;
    std::vector<std::vector<float>> insert_points;

    for (int i=0; i<cloud->points.size(); i++)
    {
        insert_points[i].push_back(cloud->points[i].x);
        insert_points[i].push_back(cloud->points[i].y);
        insert_points[i].push_back(cloud->points[i].z);
        tree->insert(insert_points[i],i);
    }

    clusters = euclideanCluster(cloud, tree, clusterTolerance); */

    

    //pcl version
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); //cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (pcl::PointIndices getIndices : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (int index : getIndices.indices)
            cloud_cluster->points.push_back (cloud->points[index]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
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

/*template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int i, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &processed, MyKdTree* tree, float distanceTol)
{
    processed[i] = true;
    cluster.push_back(i);
    std::vector<int> nearby = tree->search(cloud,distanceTol, i);
    for (int index : nearby)
    {
        if (!processed[index])
        {
            clusterHelper(index, cloud, cluster, processed, tree, distanceTol);
        }
    }
}*/

/*template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, MyKdTree* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> processed(cloud->points.size(), false);

    int i = 0;
    while(i < cloud->points.size())
    {
        if(processed[i])
        {
            i++;
            continue;
        }

        std::vector<int> cluster;
        //Proximity
        clusterHelper(i, cloud, cluster, processed, tree, distanceTol);
        clusters.push_back(cluster);
        i++;
    }
    return clusters;

}*/

/*template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> CreateData(std::vector<std::vector<int>> points)
{
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    
    for(int i = 0; i < points.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = points[i][0];
        point.y = points[i][1];
        point.z = points[i][2];;

        cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;

}*/