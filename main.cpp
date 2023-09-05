#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {
    // Load input pcd files
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/khan/code/NDT_test/PCD/202307271116_lidar[000060].pcd", *target_cloud) == -1) {
        PCL_ERROR("Couldn't read file target.pcd \n");
        return (-1);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/khan/code/NDT_test/PCD/202307271116_lidar[000063].pcd", *input_cloud) == -1) {
        PCL_ERROR("Couldn't read file input.pcd \n");
        return (-1);
    }

    // Apply NDT algorithm
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setInputSource(input_cloud);
    ndt.setInputTarget(target_cloud);
    ndt.setMaximumIterations(35);
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
              << " score: " << ndt.getFitnessScore() << std::endl;

    // Visualize the result
    pcl::visualization::PCLVisualizer viewer("NDT example");
    viewer.addPointCloud<pcl::PointXYZ>(target_cloud, "target");
    viewer.addPointCloud<pcl::PointXYZ>(output_cloud, "output");
    viewer.spin();

    return 0;
}
