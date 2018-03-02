#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

// Plane segmentation specific includes
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// ExtraxtIndices includes
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

int MAX_SURFACES = 4;
std::vector<ros::Publisher> pub(MAX_SURFACES);

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *input_cloud);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, num_points = (int)input_cloud->points.size();
  pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_cloud(
      new pcl::PointCloud<pcl::PointXYZ>),
      outliers_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 ros_inliers_cloud;
  // While 30% of the original cloud is still there
  while (input_cloud->points.size() > 0.3 * num_points and i < MAX_SURFACES) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      std::cerr << "Could not estimate a planar model for the given dataset."
                << std::endl;
      // break;
      return;
    }

    // Extract the inliers
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*inliers_cloud); // inliers_cloud now has the inliers of the
                                    // extracted plane
    std::cerr << "PointCloud representing the planar component: "
              << inliers_cloud->width * inliers_cloud->height << " data points."
              << std::endl;

    // Convert to ROS data
    pcl::toROSMsg(*inliers_cloud, ros_inliers_cloud);
    // Publish the data
    pub[i].publish(ros_inliers_cloud);

    // // Create the filtering object
    extract.setNegative(true);
    extract.filter(*outliers_cloud);  // outliers_cloud now has the outliers of
                                      // the extracted plane
    input_cloud.swap(outliers_cloud); // With this swap, now "cloud" is formed
                                      // by the outliers.
    i++;
  }

  //  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
  //            << coefficients->values[1] << " " << coefficients->values[2] <<
  //            " " << coefficients->values[3] << std::endl;

  //  std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
  //  for (size_t i = 0; i < inliers->indices.size(); ++i)
  //    std::cerr << inliers->indices[i] << "    "
  //              << cloud->points[inliers->indices[i]].x << " "
  //              << cloud->points[inliers->indices[i]].y << " "
  //              << cloud->points[inliers->indices[i]].z << std::endl;
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "plane_segmentation_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe(
      "/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  for (int i = 0; i < MAX_SURFACES; ++i) {
    std::string str = "segmented_plane/0";
    str[16] = '0' + i;
    pub[i] = nh.advertise<sensor_msgs::PointCloud2>(str, 1);
  }

  // Spin
  ros::spin();
}
