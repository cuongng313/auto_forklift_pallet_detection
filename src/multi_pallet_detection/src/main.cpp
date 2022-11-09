#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "plane_segmentation.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

PointCloud g_pcl_point_cloud;

sensor_msgs::PointCloud2 g_raw_pointcloud;

PlaneSegmentation plane_segement;

HybridRGSegmentationParameters parameters;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  g_raw_pointcloud = *msg;
  pcl::fromROSMsg(*msg, *plane_segement.point_cloud_);

  std::cout << "raw point size w and h: " << g_raw_pointcloud.width
                                          << " "
                                          << g_raw_pointcloud.height << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  // Eigen::Matrix4f trans_matrix;
  // trans_matrix << 1, 0, 0, 0,
  //                 0, cos(-M_PI), -sin(-M_PI), 0,
  //                 0, sin(-M_PI), cos(-M_PI), 0,
  //                 0, 0, 0, 1;

  // pcl::transformPointCloud(*plane_segement.point_cloud_, *temp_trans_cloud, trans_matrix);
  // plane_segement.point_cloud_ = temp_trans_cloud;

  // std::vector<int> indices; 
  // pcl::removeNaNFromPointCloud(*plane_segement.point_cloud_, *temp_cloud, indices);
  // plane_segement.point_cloud_ = temp_cloud;

  pcl::VoxelGrid<pcl::PointXYZ> sor;

  std::cout << "point filtered size " << plane_segement.point_cloud_->size() << std::endl;
  std::cout << "point filtered is organized: " << plane_segement.point_cloud_->isOrganized() << std::endl;
  std::cout << "point filtered frame id: " << plane_segement.point_cloud_->header.frame_id << std::endl;
  std::cout << "point filtered height: " << plane_segement.point_cloud_->height << std::endl;
  std::cout << "point filtered width: " << plane_segement.point_cloud_->width << std::endl;
  

  // Transform from camera_depth_optical_frame (z-forward) 
  // to camera_frame (y-forward, x-right, z-up)

  parameters.limit_dis = 4.0; // distance limit (m)
  parameters.max_mass2plane_dis = 0.05;
  parameters.min_dot_product = 0.95;
  parameters.max_segment_mse = 0.01;
  parameters.min_segment_size = 1000;
  parameters.subwindow_side_length = 3;
  parameters.valid_neighbor_cnt = 3; // the minimum neighbors that the segment can be considered


  if (plane_segement.point_cloud_->size() != 0)
  {
    plane_segement.setInput(plane_segement.point_cloud_);
    plane_segement.setParameter(parameters);
    plane_segement.preProcessing();

    std::cout << "**** Points after processing ****" << std::endl;
    std::cout << "Point size: " << plane_segement.points_.size() << std::endl;
  


    plane_segement.applySegmentation();

    
    // for (int i = 30000; i <= 30500; i++)
    // {
    //   std::cout << "point " << i << ":" 
    //             << std::endl << plane_segement.points_.at(0) << std::endl;
    // }

    if(plane_segement.planar_patches_.size() != 0)
    {
      plane_segement.colorEncoding(output, false);
      pcl::visualization::PCLVisualizer viewer ("Detected planes with Pseudo-color");
      
      viewer.setBackgroundColor (1.0, 1.0, 1.0);

      viewer.addPointCloud (output, "segments");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segments");
      viewer.spin();
    }
    std::cout << std::endl;
  }
  

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_segmentation");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);

  /* ROS Subscriber */
  // ros::Subscriber sub_point_cloud = n.subscribe("point_cloud/points", 10, pointCloudCallback);

  ros::Subscriber sub_point_cloud = n.subscribe("/camera/depth/color/points", 10, pointCloudCallback);

  /* ROS Publisher */
  ros::Publisher pub_pcl_pointcloud = n.advertise<PointCloudRGB>("pcl_point_cloud", 1);



  while (ros::ok()) 
  {

    // output->header.frame_id = "camera_depth_optical_frame" ;
    // pub_pcl_pointcloud.publish(*output);
    pub_pcl_pointcloud.publish(*plane_segement.point_cloud_);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}