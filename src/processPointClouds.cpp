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
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered ( new pcl::PointCloud<PointT>);
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);// define size of the cube that we are working with 
    sor.filter (*cloud_filtered);

    // define region that we are intersted in , which contain road and the obstacle     
    typename pcl::PointCloud<PointT>::Ptr cloud_Region ( new pcl::PointCloud<PointT>);
    //CropBox is a filter that allows the user to filter all the data inside of a given box.
    pcl::CropBox<PointT> region(true); // true as we deal with the points inside the box
    //define the box region using minPoint and maxPoint
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    // save result in cloud_Region (points inside the box region) and any point out side this box will removed 
    region.filter(*cloud_Region); 

    // remove the points in the roof of the car 
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true); 
    region.setMin(Eigen::Vector4f(-1.5 , -1.7 , -1 , 1));
    region.setMax(Eigen::Vector4f(2.6 , 1.7 , -.4 , 1));
    region.setInputCloud(cloud_Region);
    region.filter(indices); // indices of points inside cloud_Region that fit inside the box 

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices}; // to store the indices of the roop points 
    for(int point : indices)
        inliers->indices.push_back(point); 

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_Region);
    extract.setIndices(inliers);
    extract.setNegative(true); // remove points with indices in inliers from the cloud_region 
    extract.filter(*cloud_Region);  // store result in cloud_region

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_Region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr plane_cloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud (new pcl::PointCloud<PointT>());
    /* to extact the plane cloud from the cloud 
    looping over inliers and get the points of these indices from the cloud and write it to the plane_cloud*/
    for(int index : inliers->indices){
        plane_cloud->points.push_back(cloud->points[index]);
    }
    /*To generate the obstacle cloud
     one way to use PCL to do this is to use an extract object,
     which subtracts the plane cloud from the input cloud.  */ 
     // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacle_cloud); 
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    // create pointIndices which is a vector of indices of points that in the Road plane (Non obstacle plane)
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // create coefficients of our model /* use it if we want to reder this plane in the PCL viewer*/
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients ()};
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    // Segment the largest planar component from the  cloud
    seg.setInputCloud (cloud); // we want to do segmentation in our cloud 
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    // call SeparateClouds which separate the points of the Road plane and the points of the obstacle plane 
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    // define cluster vector which contain a vetor of clusters in the cloud 
    // each cluster is a pointCloud 
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    std::vector<pcl::PointIndices> cluster_indices;
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    // go through cluster indices and create some point clouds in each point cloud is going to be a different cluster
    // pcl::PointIndices is vector of the Pcl points indices 
    for(pcl::PointIndices getIndices : cluster_indices){
        // create a new cluster 
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for(int index : getIndices.indices){
        cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height=1;
        cloudCluster->is_dense = true ;
        clusters.push_back(cloudCluster);
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
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{   
    auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;

	// TODO: Fill in this function
	while(maxIterations--){
		// create inliers as a unordered set to store the index of some point of cloud
		std::unordered_set<int> inliers;
		// to do not choose the two random numbers with the same value as the set will not insert a repeated value 
		// pick 3 points randomly among the cloud points
		while(inliers.size()<3)	
			inliers.insert(rand()%(cloud->points.size()));

		float x1,x2,x3,y1,y2,y3,z1,z2,z3;
		//Return an iterator pointing to the first element in the unordered_set container.
		auto itr = inliers.begin();
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

		//std::vector<float> v1 {x2-x1 , y2-y1 , z2-z1}; //Vector v1 travels from point1 to point2.
		//std::vector<float> v2 {x3-x1 , y3-y1 , z3-z1}; //Vector v2 travels from point1 to point3.
		float i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);

		float A = i;
		float B = j;
		float C = k;
		float D = -(i*x1+j*y1+k*z1);

		for(int index=0 ; index < cloud->points.size() ; index++){
			if(inliers.count(index)>0)
				continue;
			cloud->points[index];
			float x0 = cloud->points[index].x;
			float y0 = cloud->points[index].y;
			float z0 = cloud->points[index].z;

			float d = fabs(x0*A + y0*B + z0*C + D)/sqrt((A*A)+(B*B)+(C*C));
			// if distance is less than the theshold , then inset this index in the inliers set 
			if(d <= distanceTol){
				inliers.insert(index); 
			}
			if(inliers.size()>inliersResult.size()){
				inliersResult = inliers;
			}
			
			}

	}
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
	// For max iterations 
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}


template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int pointIndex ,typename pcl::PointCloud<PointT>::Ptr cloud , std::vector<bool>& processed , typename KdTree<PointT>::KdTree* tree , std::vector<int> & cluster ,  float distanceTol, int maxSize){
	if((processed[pointIndex] ==false)&&(cluster.size()<maxSize)){ // check on cluster Max numbers of points 
    processed[pointIndex]=true;
	cluster.push_back(pointIndex);
	// search for nearby points which is near to this point to add them to this cluster
	 // call search to get the indices of the nearby points 
	std::vector<int> nearset = tree->search(cloud->points[pointIndex],distanceTol);
	// iterate each nearby point 
	for(int nearID : nearset){
		if(!processed[nearID])
		clusterHelper(nearID , cloud , processed , tree , cluster ,distanceTol,maxSize);
    	}
    }
}
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster_indices(typename pcl::PointCloud<PointT>::Ptr cloud, typename KdTree<PointT>::KdTree* tree, float distanceTol, int minSize, int maxSize){
    std::vector<bool> processed(cloud->points.size(),false);
    std::vector<std::vector<int>> clusters;

    for(int pointIndex=0;pointIndex < cloud->points.size();pointIndex++){
        if(processed[pointIndex]!=true){ // point is not processed
        std::vector<int> cluster;
        clusterHelper(pointIndex,cloud,processed , tree , cluster ,distanceTol,maxSize);
        if((cluster.size()< maxSize)&&(cluster.size()> minSize))
        clusters.push_back(cluster);
        }
        
    }
    return clusters;
}
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanClusterMain(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize){
    auto startTime = std::chrono::steady_clock::now();

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters; // Vector of clusters 

    typename KdTree<PointT>::KdTree* tree = new KdTree<PointT>;
    tree->insertCloud(cloud);

    std::vector<std::vector<int>> cluster_indices = euclideanCluster_indices(cloud,tree,distanceTol,minSize,maxSize);
    int clusterId = 0;
	for(std::vector<int> cluster : cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for(int ID : cluster){
            clusterCloud->points.push_back(cloud->points[ID]);
        } 
        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;

        clusters.push_back(clusterCloud);
        //renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
        //clusterId++;
         auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
	}
	return clusters;
}