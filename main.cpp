#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/octree/octree.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <vtkSmartPointer.h>
#include <vtkStaticPointLocator.h>

#include <chrono>
#include <random>

int main(int argc, const char* argv[])
{
    int numLookups = 5000; //200000;
    int numNeighbors = 5;
    std::string filename = "bun000.ply";

    // Prepare PCL data structures
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader reader;
    reader.read(filename, *cloud);
    std::cout << "PCL read " << cloud->size() << " bunny points" << std::endl;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.01);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    std::cout << "PCL octree size: " << octree.getLeafCount() << std::endl;

    double min[3];
    double max[3];
    octree.getBoundingBox(min[0], min[1], min[2], max[0], max[1], max[2]);

    // Prepare VTK data structures
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::pointCloudTovtkPolyData(*cloud, polyData);
    std::cout << "VTK read " << polyData->GetNumberOfPoints() << " bunny points" << std::endl;

    vtkSmartPointer<vtkStaticPointLocator> locator = vtkSmartPointer<vtkStaticPointLocator>::New();
    locator->SetTolerance(0.01);
    //locator->SetDivisions(500, 500, 500);
    //locator->SetNumberOfPointsPerBucket(1);
    locator->SetDataSet(polyData);
    locator->BuildLocator();
    std::cout << "VTK locator size: " << locator->GetNumberOfBuckets() << std::endl;

    std::vector<int> indices;
    std::vector<float> dist2;
    pcl::PointXYZ testPoint;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);

    std::chrono::steady_clock::time_point begin, end;

    // Run PCL tests
    begin = std::chrono::steady_clock::now();
    for(size_t i = 0; i < numLookups; i++)
    {
        testPoint.x = min[0] + dis(gen) * (max[0] - min[0]);
        testPoint.y = min[1] + dis(gen) * (max[1] - min[1]);
        testPoint.z = min[2] + dis(gen) * (max[2] - min[2]);
        octree.nearestKSearch(testPoint, numNeighbors, indices, dist2);
    }
    end = std::chrono::steady_clock::now();

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0;
    std::cout << "PCL did " << numLookups << " in " << diff << std::endl;

    // Run VTK tests
    vtkSmartPointer<vtkIdList> results = vtkSmartPointer<vtkIdList>::New();
    begin = std::chrono::steady_clock::now();
    double vtkTestPoint[3];
    for(size_t i = 0; i < numLookups; i++)
    {
        vtkTestPoint[0] = min[0] + dis(gen) * (max[0] - min[0]);
        vtkTestPoint[1] = min[1] + dis(gen) * (max[1] - min[1]);
        vtkTestPoint[2] = min[2] + dis(gen) * (max[2] - min[2]);
        locator->FindClosestNPoints(numNeighbors, vtkTestPoint, results);
    }
    end = std::chrono::steady_clock::now();

    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0;
    std::cout << "VTK did " << numLookups << " in " << diff << std::endl;

    /*pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {

    }*/

    return 0;
}
