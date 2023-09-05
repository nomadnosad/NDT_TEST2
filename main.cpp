#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {
    // Load input pcd files
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/khan/DATA/PCD_pangyo_20230727/202307271116_lidar[000060].pcd", *target_cloud) == -1) {
        PCL_ERROR("Couldn't read file target.pcd \n");
        return (-1);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/khan/DATA/PCD_pangyo_20230727/202307271116_lidar[000061].pcd", *input_cloud) == -1) {
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

    // Add input_cloud to the viewer and set its color to green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(input_cloud, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(input_cloud, green, "source");

    // Add target_cloud to the viewer and set its color to yellow
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> yellow(target_cloud, 255, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(target_cloud, yellow, "target");

    // Add output_cloud to the viewer and set its color to blue
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(output_cloud, 0, 0, 255);
    viewer.addPointCloud<pcl::PointXYZ>(output_cloud, blue, "output");

    viewer.spin();


    return 0;
}
