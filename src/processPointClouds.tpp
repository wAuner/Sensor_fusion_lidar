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

// performs downsampling and crops the region of interest of the input cloud
// also removes points detected from the car's roof
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
                                        Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Done:: Fill in the function to do voxel grid point reduction and region based filtering
    // voxel downsampling
    typename pcl::PointCloud<PointT>::Ptr sampledCloud{new pcl::PointCloud<PointT>};
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.filter(*sampledCloud);

    // throw everything outside of ROI away
    typename pcl::PointCloud<PointT>::Ptr roiCloud{new pcl::PointCloud<PointT>};
    pcl::CropBox<PointT> roi(true);
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(sampledCloud);
    roi.filter(*roiCloud);

    // filter points from car roof by inverse ROI filtering
    // collect points to be removed
    std::vector<int> roofIndices;
    pcl::CropBox<PointT> carRoofFilter(true);
    // define area of roof / where to filter
    carRoofFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1., 1.));
    carRoofFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1.));
    carRoofFilter.setInputCloud(roiCloud);
    carRoofFilter.filter(roofIndices);
    // prepare separation of roof points
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices()};
    for (int roofPoint : roofIndices) {
        inliers->indices.push_back(roofPoint);
    }

    // extract roof points from filtered cloud
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(roiCloud);
    extractor.setIndices(inliers);
    extractor.setNegative(true);
    extractor.filter(*roiCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return roiCloud;

}

// separates the input cloud into two and returns them as pair
// helper gets called from SegmentPlane, could be private
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers,
                                           typename pcl::PointCloud<PointT>::Ptr cloud) {

    // Done: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr roadCloud(new pcl::PointCloud<PointT>());

    // add inliers to roadCloud
    for (int index : inliers->indices) {
        roadCloud->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> extractor;
    // which cloud to extract from
    extractor.setInputCloud(cloud);
    // which indices of the cloud to extract
    extractor.setIndices(inliers);
    // mark specified indices to extract, inliers will be removed
    extractor.setNegative(true);
    // copy points to other cloud
    extractor.filter(*obstacleCloud);

    // TODO: find out why this isnt working
    //auto segResult = std::make_pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(obstacleCloud, roadCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud,
                                                                                                      roadCloud);
    return segResult;
}

// Segments the cloud into a cloud containing the road points and one containing everything else
// by using pcl's Ransac implementation
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
                                         float distanceThreshold) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Done:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setModelType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cout << "No model found" << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(
            inliers, cloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

// finds clusters within the obstacle cloud to detect objects
// uses pcl's euclidean cluster algorithm
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize,
                                       int maxSize) {

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Done:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    // each element contains a set of indices which belongs to a specific cluster
    std::vector<pcl::PointIndices> vec_cluster_indices;

    pcl::EuclideanClusterExtraction<PointT> extractor;
    extractor.setClusterTolerance(clusterTolerance);
    extractor.setMinClusterSize(minSize);
    extractor.setMaxClusterSize(maxSize);
    extractor.setSearchMethod(tree);
    extractor.setInputCloud(cloud);
    extractor.extract(vec_cluster_indices);

    // now distribute points to create a cloud for each cluster
    for (const pcl::PointIndices& cluster_indices : vec_cluster_indices) {
        // separate cloud for each cluster
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
        for (int indice : cluster_indices.indices) {
            // take the indices from the main cloud and copy the points to the
            // cluster cloud
            cluster->points.push_back(cloud->points.at(indice));
        }
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        // collect cluster cloud to return them
        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
              << " clusters" << std::endl;

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