#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include "../include/Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include <unordered_set>
#include "../include/tree_utilities.hpp"
#include <filesystem>
namespace fs = std ::filesystem;

#define USE_PCL_LIBRARY
using namespace lidar_obstacle_detection;

typedef std::unordered_set<int> my_visited_set_t;

// This function sets up the custom kdtree using the point cloud
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree *tree, int dimension)
{
    // insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
    {
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
    }
}

/*
OPTIONAL
This function computes the nearest neighbors and builds the clusters
    - Input:
        + cloud: Point cloud to be explored
        + target_ndx: i-th point to visit
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters
        + visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
        + cluster: Here we add points that will represent the cluster
        + max: Max cluster size
    - Output:
        + visited: already visited points
        + cluster: at the end of this function we will have one cluster
*/
void proximity(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree *tree, float distanceTol, my_visited_set_t &visited, std::vector<int> &cluster, int max)
{
    if (cluster.size() < max)
    {
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point{cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};

        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree->search(point, distanceTol);

        for (int neighborNdx : neighborNdxs)
        {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end())
            {
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.size() >= max)
            {
                return;
            }
        }
    }
}

/*
OPTIONAL
This function builds the clusters following a euclidean clustering approach
    - Input:
        + cloud: Point cloud to be explored
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters
        + setMinClusterSize: Minimum cluster size
        + setMaxClusterSize: Max cluster size
    - Output:
        + cluster: at the end of this function we will have a set of clusters
TODO: Complete the function
*/
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree *tree, float distanceTol, int setMinClusterSize, int setMaxClusterSize)
{
    my_visited_set_t visited{};              // already visited points
    std::vector<pcl::PointIndices> clusters; // vector of PointIndices that will contain all the clusters
    std::vector<int> cluster;                // vector of int that is used to store the points that the function proximity will give me back
    // for every point of the cloud
    for (int i = 0; i < cloud->size(); i++)
    {
        //   if the point has not been visited (use the function called "find")
        if (visited.find(i) == visited.end())
        {
            //     find clusters using the proximity function
            pcl::PointIndices Cluster_indices;
            proximity(cloud, i, tree, distanceTol, visited, cluster, setMaxClusterSize);

            // if we have more clusters than the minimum
            if (cluster.size() >= setMinClusterSize)
            {
                //       Create the cluster and insert it in the vector of clusters. You can extract the indices from the cluster returned by the proximity funciton (use pcl::PointIndices)
                Cluster_indices.indices = cluster;
                clusters.push_back(Cluster_indices);
                //     end if
            }
            //   end if
        }
        // end for
    }
    return clusters;
}

void ProcessAndRenderPointCloud(Renderer &renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // TODO: 1) Downsample the dataset
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    // Voxel Filtering
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*cloud_filtered);
    // [DEBUG] output.
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl;

    // here we crop the points that are far away from us, in which we are not interested
    pcl::CropBox<pcl::PointXYZ> cb(true);
    cb.setInputCloud(cloud_filtered);
    cb.setMin(Eigen::Vector4f(-20, -6, -2, 1));
    cb.setMax(Eigen::Vector4f(30, 7, 5, 1));
    cb.filter(*cloud_filtered);

    // TODO: 3) Segmentation and apply RANSAC
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    // play with this number to obtain stable point cloud
    seg.setDistanceThreshold(0.3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    int i = 0, nr_points = (int)cloud_filtered->size();

    while (cloud_filtered->size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

        // Remove the planar inliers, extract the rest.
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }
    // TODO: 4) iterate over the filtered cloud, segment and remove the planar inliers

#ifdef USE_PCL_LIBRARY
    // TODO: 5) Create the KDTree and the vector of PointIndices
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);
    // Make the Euclidean clustering
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.3); // Change the parameters and see what happen
    // Set the range of the cluster
    ec.setMinClusterSize(200);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    // TODO: 6) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!)
    std::vector<pcl::PointIndices> cluster_indices;
    // extracting the indices
    ec.extract(cluster_indices);
    // PCL functions
    // HERE 6)
#else
    // Optional assignment
    my_pcl::KdTree treeM;
    treeM.set_dimension(3);
    setupKdtree(cloud_filtered, &treeM, 3);
    cluster_indices = euclideanCluster(cloud_filtered, &treeM, clusterTolerance, setMinClusterSize, setMaxClusterSize);
#endif

    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1), Color(1, 0, 1), Color(0, 1, 1)};

    /**Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices.

    To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
    Compute euclidean distance
    **/
    int j = 0;
    int clusterId = 0;
    // Calculate the distance from the centroid to the ego vehicle
    float ego_vehicle_x = 0.0; // Ego vehicle assumed at the origin
    float ego_vehicle_y = 0.0; // Ego vehicle assumed at the origin
    float ego_vehicle_z = 0.0; // Ego vehicle assumed at the origin
    // define the distance float variable
    float distance;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->push_back((*cloud_filtered)[*pit]);
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // renderer.RenderPointCloud(cloud, "originalCloud" + std::to_string(clusterId), colors[2]);
        // TODO: 7) render the cluster and plane without rendering the original cloud
        renderer.RenderPointCloud(cloud_plane, "planeCloud" + std::to_string(clusterId), Color(0, 1, 0));
        renderer.RenderPointCloud(cloud_cluster, "clusterCloud" + std::to_string(clusterId), colors[2]);

        //----------

        // Here we create the bounding box on the detected clusters
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

        // TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle
        Box box{minPt.x, minPt.y, minPt.z,
                maxPt.x, maxPt.y, maxPt.z};

        // TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle
        // please take a look at the function RenderBox to see how to color the box

        // Calculate the centroid of the cluster
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);

        // Calculate the distance from the centroid to the assumed ego vehicle position
        distance = sqrt((centroid[0] - ego_vehicle_x) * (centroid[0] - ego_vehicle_x) + (centroid[1] - ego_vehicle_y) * (centroid[1] - ego_vehicle_y) + (centroid[2] - ego_vehicle_z) * (centroid[2] - ego_vehicle_z));

        // use addTExt defined in the renderer.cpp where the command addText3D is defined
        renderer.addText(centroid[0], centroid[1], centroid[2], std::to_string(distance) + " m");

        // If loop with condition the distance with respect to the ego_vehicle to change the color with the variation of distance.
        if (distance <= 5.0)
        {
            // Render the bounding box of the cluster with red color
            renderer.RenderBox(box, j, Color(1, 0, 0));
        }
        else
        {
            // Render the bounding box of the cluster with the green color
            renderer.RenderBox(box, j, colors[4]);
        }
        ++clusterId;
        j++;
    }
}
// main remember to change the path
int main(int argc, char *argv[])
{
    Renderer renderer;
    renderer.InitCamera(CameraAngle::XY);
    // Clear viewer
    renderer.ClearViewer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    /*std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"Users/seuffoakouanhangoume/Documents/EEIV/Autonomous_Driving/dataset_1"},
                                                boost::filesystem::directory_iterator{});
    */
    std::vector<fs::path> stream;
    for (const auto &entry : fs::directory_iterator("/Users/seuffoakouanhangoume/Documents/EEIV/Autonomous_Driving/dataset_1/"))
    {
        stream.push_back(entry.path());
    }

    // sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());

    auto streamIterator = stream.begin();

    while (not renderer.WasViewerStopped())
    {
        renderer.ClearViewer();

        pcl::PCDReader reader;
        reader.read(streamIterator->string(), *input_cloud);
        auto startTime = std::chrono::steady_clock::now();

        ProcessAndRenderPointCloud(renderer, input_cloud);
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
                  << input_cloud->points.size() << " data points from " << streamIterator->string() << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        renderer.SpinViewerOnce();
    }
}
