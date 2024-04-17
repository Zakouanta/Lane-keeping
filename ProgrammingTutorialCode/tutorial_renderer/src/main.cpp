#include "Renderer.hpp"
#include <chrono>
#include <iostream>
#include <random>
#include <sstream>
#include <vector>
#include <fstream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
using namespace Rendering;
using namespace std;


int main(int argc, char* argv[])
{
        
    Renderer renderer;
    // Initializes the point of vieweer
    renderer.InitCamera(CameraAngle::XY);
    renderer.ClearViewer();

    // Cloud structure used from PCL to visualize the point cloud (not important now)
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Just use boost to iterate in the folder (if you have multiple files this is useful)
	std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"/home/thrun/Desktop/Datasets/log_renderer"},
	boost::filesystem::directory_iterator{});

    // Sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());
    auto streamIterator = stream.begin();

    while (not renderer.WasViewerStopped())
    {
        // Clear the renderer before rendering a new cloud
        renderer.ClearViewer();

        pcl::PCDReader reader;
        reader.read (streamIterator->string(), *input_cloud);

        // Start the timer 
        auto startTime = std::chrono::steady_clock::now();

        // Declare the color of the point cloud
        Color c(0.1,0.1,0.1);

        // TODO: Here it is defined the cloud to be rendered (using renderer and renderpointcloud function)
        
        // TODO: Define a bounding box

        // TODO: Add to the render queue the bounding box

        // TODO: Add to the render queue a 2D circle

        // TODO: Add to the render queue a text

        // Iterate to the next file
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        // Here the cloud is really rendered
        renderer.SpinViewerOnce();

        auto endTime = std::chrono::steady_clock::now();
        // COmpute the time taken to render the cloud
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
                  << input_cloud->points.size() << " data points from " << streamIterator->string() <<  "rendering took " << elapsedTime.count() << " milliseconds" << std::endl;

    }
}

