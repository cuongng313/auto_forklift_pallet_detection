#include <ros/ros.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>

#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

/* Plane Segmentation */
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/filters/conditional_removal.h> 
#include <pcl/visualization/cloud_viewer.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"

#include <dynamic_reconfigure/server.h>
#include <multi_pallet_detection/palletDetectionReconfigureConfig.h>

/* type define */
typedef pcl::PointCloud<pcl::PointXYZ> pcl_PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> pcl_PointCloudRGB;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudN;
typedef pcl_msgs::PointIndices PCLIndicesMsg;

typedef multi_pallet_detection::palletDetectionReconfigureConfig Config;

class PalletDetection
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_filtered_pointcloud_;
    ros::Publisher pub_ground_filtered_pointcloud_;
    ros::Publisher pub_pallet_pointcloud_;
    ros::Publisher pub_rgb_pointcloud;
    ros::Publisher pub_pallet_pose_;
    ros::Publisher pub_pallet_pose_kalman_;
    ros::Publisher pub_pallet_gt_;

    ros::Subscriber sub_point_cloud_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    
    /* Pointcloud cutoff */
    double min_x_cutoff_;
    double max_x_cutoff_;
    double min_y_cutoff_;
    double max_y_cutoff_;
    double min_z_cutoff_;
    double max_z_cutoff_;
    int downsample_scale_ = 2;

    /* Pointcloud remove floor */
    double floor_distance_thresh_;
    bool active_remove_floor_;

    /* Pointcloud segementation */
    bool active_segment_;
    bool opencv_show_;
    double min_cluster_size_;
    double max_cluster_size_;
    double neighbor_number_;
    double curvature_thres_;
    double smooth_thresh_;
    double k_search_;
    
    /* Pointcloud normal estimation */
    int normal_estimation_method_;
    bool border_policy_ignore_;
    bool publish_normal_;
    double normal_smoothing_size_;
    double max_depth_change_factor_;
    bool depth_dependent_smoothing_;

    /* Template matching */
    std::string template_directory_;
    cv::Mat templ_img_;
    cv::Mat source_img_;
    cv::Mat result_img_;
    int match_method_{0};
    bool template_matching_show_;
    double scale_par_;
    double min_scale_;
    double max_scale_;
    bool manual_scale_;
    double thres_;
    double optimal_scale_{0.0}, last_optimal_scale_{0.0};
    double change_thres_;
    double scale_change_rate_;
    bool tm_quality_;

    /* Kalman filter */
    Eigen::Vector3d init_pose_;
    Eigen::Matrix3d init_cov_;
    Eigen::Vector3d tracking_pose_;
    Eigen::Matrix3d tracking_cov_;
    double process_noise_;
    double measure_noise_;
    double pallet_gt_x_;
    double pallet_gt_y_;
    double pallet_gt_yaw_;

    /* Common variables */
    pcl_PointCloud cloud_;
    pcl_PointCloud cloud_f_;
    pcl_PointCloudRGB cloud_rgb_;
    sensor_msgs::Image pallet_image_;
    pcl::PointXY pallet_point_xy_;
    geometry_msgs::PoseStamped pallet_pose_;
    geometry_msgs::PoseStamped pallet_pose_output_;
    geometry_msgs::PoseStamped pallet_pose_gt_;
    double pallet_yaw_angle_;

    bool init_reconfig_ = true;

public:

    PalletDetection(ros::NodeHandle &paramGet) 
    {
        /* Get Param */
        /* Cut off distnace */
        paramGet.param<double>("min_x_cutoff", min_x_cutoff_, -0.7);
        paramGet.param<double>("max_x_cutoff", max_x_cutoff_, 0.7);
        paramGet.param<double>("min_y_cutoff", min_y_cutoff_, -0.7);
        paramGet.param<double>("max_y_cutoff", max_y_cutoff_, 0.7);
        paramGet.param<double>("min_z_cutoff", min_z_cutoff_, 2.2);
        paramGet.param<double>("max_z_cutoff", max_z_cutoff_, 4.0);
        paramGet.param<int>("downsample_scale", downsample_scale_, 2);

        /* Ground filter */
        paramGet.param<bool>("active_remove_floor", active_remove_floor_, true);
        paramGet.param<double>("floor_distance_thresh", floor_distance_thresh_, 0.01);

        /* Plane segmentation */
        paramGet.param<bool>("active_segment", active_segment_, true);
        paramGet.param<bool>("opencv_show", opencv_show_, false);
        paramGet.param<double>("min_cluster_size", min_cluster_size_, 1000);
        paramGet.param<double>("max_cluster_size", max_cluster_size_, 1000000);
        paramGet.param<double>("neighbor_number", neighbor_number_, 50);
        paramGet.param<double>("curvature_thres", curvature_thres_, 50);
        paramGet.param<double>("smooth_thresh", smooth_thresh_, 2);        //angle degree
        paramGet.param<double>("k_search", k_search_, 20);      

        /* Normal estimation */
        paramGet.param<int>("normal_estimation_method", normal_estimation_method_, 1);
        paramGet.param<bool>("border_policy_ignore", border_policy_ignore_, false);
        paramGet.param<bool>("publish_normal", publish_normal_, true);
        paramGet.param<double>("normal_smoothing_size", normal_smoothing_size_, 20.0);
        paramGet.param<double>("max_depth_change_factor", max_depth_change_factor_, 0.02);
        paramGet.param<bool>("depth_dependent_smoothing", depth_dependent_smoothing_, false);

        /* template matching */
        paramGet.param<std::string>("template_directory", template_directory_, "");
        paramGet.param<int>("match_method", match_method_, 0);
        paramGet.param<bool>("template_matching_show", template_matching_show_, false);
        paramGet.param<double>("scale_par", scale_par_, 1.0);
        paramGet.param<double>("min_scale", min_scale_, 0.9);
        paramGet.param<double>("max_scale", max_scale_, 1.6);
        paramGet.param<bool>("manual_scale", manual_scale_, false);
        paramGet.param<double>("thres", thres_, 1.0);
        paramGet.param<double>("change_thres", change_thres_, 0.2);
        paramGet.param<double>("scale_change_rate_", scale_change_rate_, 0.01);

        templ_img_ = cv::imread(template_directory_, cv::IMREAD_GRAYSCALE);

        /* kalman filter */
        paramGet.param<double>("process_noise", process_noise_, 0.001);
        paramGet.param<double>("measure_noise", measure_noise_, 0.02);
        paramGet.param<double>("pallet_gt_x", pallet_gt_x_, 0.02);
        paramGet.param<double>("pallet_gt_y", pallet_gt_y_, 0.02);
        paramGet.param<double>("pallet_gt_yaw", pallet_gt_yaw_, 0.02);

        /* ROS Subscriber */
        sub_point_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, 
                                                        &PalletDetection::pointCloudCallback, this);

        /* ROS Publisher */
        pub_filtered_pointcloud_ = nh_.advertise<pcl_PointCloudRGB>("pd_filtered_pointcloud", 1);
        pub_ground_filtered_pointcloud_ = nh_.advertise<pcl_PointCloudRGB>("pd_ground_filtered_pointcloud", 1);
        pub_pallet_pointcloud_ = nh_.advertise<pcl_PointCloudRGB>("pd_pallet_pointcloud", 1);
        pub_rgb_pointcloud = nh_.advertise<pcl_PointCloudRGB>("pd_pallet_rgb_pointcloud", 1);

        // Pallet pose publisher
        pub_pallet_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("pd_pallet_pose", 1);
        pub_pallet_pose_kalman_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pd_pallet_pose_kalman", 1);
        pub_pallet_gt_ = nh_.advertise<geometry_msgs::PoseStamped>("pd_pallet_pose_gt", 1);

        // dynamic reconfigure server
        srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (paramGet);
        dynamic_reconfigure::Server<Config>::CallbackType f;
        f = boost::bind(&PalletDetection::reconfigCallback, this, _1, _2);
        srv_->setCallback(f);

        // Pose in the coordinate of camera, ignore y, then only have the coordinate of x, z, pitch
        init_pose_.x() = 0; init_pose_.y() = 2; init_pose_.z() = M_PI/2;
        init_cov_.setIdentity(); init_cov_(0,0) = 10; init_cov_(1,1) = 10; init_cov_(2,2) = 10;
    
        tracking_pose_ = init_pose_;
        tracking_cov_ = init_cov_;
    }

    void filterPointCloud(pcl_PointCloud &input, pcl_PointCloud &output) 
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(input.makeShared());
        pass.setFilterFieldName ("z");
        pass.setFilterLimits(min_z_cutoff_, max_z_cutoff_);
        pass.setKeepOrganized(true);
        pass.filter (output); 

        input = output;
        pass.setInputCloud(input.makeShared());
        pass.setFilterFieldName ("x");
        pass.setFilterLimits(min_x_cutoff_, max_x_cutoff_);
        pass.setKeepOrganized(true);
        pass.filter (output); 

        input = output;
        pass.setInputCloud(input.makeShared());
        pass.setFilterFieldName ("y");
        pass.setFilterLimits(min_y_cutoff_, max_y_cutoff_);
        pass.setKeepOrganized(true);
        pass.filter (output); 

        if(downsample_scale_ > 1)
        {
            pcl_PointCloud::Ptr keypoints (new pcl_PointCloud);
            keypoints->width = output.width / downsample_scale_;
            keypoints->height = output.height / downsample_scale_;
            keypoints->points.resize(keypoints->width * keypoints->height);
            keypoints->header = output.header;
            for( size_t i = 0, ii = 0; i < keypoints->height; ii += downsample_scale_, i++)
            {
                for( size_t j = 0, jj = 0; j < keypoints->width; jj += downsample_scale_, j++)
                {
                    keypoints->at(j, i) = output.at(jj, ii);
                }
            }
            output = *keypoints;
        }  
    }

    void groundFilter(pcl_PointCloud &input, pcl_PointCloud &output)
    {
        pcl::ModelCoefficients coefficients;
        pcl::PointIndices inliers;
        
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
            
        // Optional
        seg.setOptimizeCoefficients (true);
            
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        // seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (floor_distance_thresh_);
            
        seg.setInputCloud (input.makeShared());
        seg.segment (inliers, coefficients);
            
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (input.makeShared());
        extract.setIndices (boost::make_shared<std::vector<int> > (inliers.indices));
        extract.setNegative (true);
        extract.setKeepOrganized (true);
        extract.filter (output);
    }

    void estimateNormalKdtree( const pcl_PointCloud::Ptr &input,
                            pcl::PointCloud<pcl::Normal>::Ptr output)
    {
        pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (input);
        normal_estimator.setKSearch (k_search_);
        normal_estimator.compute (*output);
    }
    
    bool planeSegementation(pcl_PointCloud &input, pcl_PointCloudRGB &output, 
                            const pcl::PointCloud<pcl::Normal>::Ptr& normals)
    {
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize (min_cluster_size_);
        reg.setMaxClusterSize (max_cluster_size_);
        pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (neighbor_number_);
        reg.setInputCloud (input.makeShared());
        //reg.setIndices (indices);
        reg.setInputNormals (normals);
        reg.setSmoothnessThreshold (smooth_thresh_ / 180.0 * M_PI);
        reg.setCurvatureThreshold (curvature_thres_);

        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);

        int cluster_size = clusters.size();

        if (cluster_size != 0)
        {
            // std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
            // std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
            pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
            colored_cloud->header.frame_id = input.header.frame_id;

            // output = *colored_cloud;

            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud (colored_cloud);
            extract.setIndices (boost::make_shared<std::vector<int> > (clusters[0].indices));
            extract.setKeepOrganized (true);
            extract.filter (output);
            return 1;
        }
        else
        {
            ROS_INFO("no cluster found!");
            return 0;
        }

    }

    /* Template Matching */
    bool templateMatchingPallet(cv::Mat img, cv::Mat templ, cv::Mat output, 
                    double &max_point, pcl::PointXY &pallet_xy)
    {
        cv::Mat img_display;
        img.copyTo(img_display);

        cv::Mat result;
        int result_cols = img.cols - templ.cols + 1;
        int result_rows = img.rows - templ.rows + 1;
        result.create(result_rows, result_cols, CV_8U);

        cv::Point matchLoc;
        cv::Point minLoc;
        cv::Point maxLoc;
        double minVal;
        double maxVal;

        // Scale template to match the size of the image
        if (manual_scale_)  optimal_scale_ = scale_par_;
        else  // auto
        {
            double max_val_optimal = 0;
            
            cv::Point min_loc_optimal;
            cv::Point max_loc_optimal;

            cv::Mat templ_scale;
            templ.copyTo(templ_scale);
            
            int original_temp_col = templ.cols;
            int original_temp_row = templ.rows;

            for(int i = min_scale_*10; i <= max_scale_*10; i++)
            {
                scale_par_ = double(i)/10;
                if (scale_par_ == 0) continue;
                cv::resize(templ, templ_scale, cv::Size(original_temp_col*scale_par_, 
                            original_temp_row*scale_par_), scale_par_, scale_par_);
                if((templ_scale.cols >= img.cols) || (templ_scale.rows >= img.rows)) break;

                cv::matchTemplate(img, templ_scale, result, match_method_);
                cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
                // ROS_INFO("scale: : %f", scale_par_);               
                // ROS_INFO("maxVal: %f", maxVal);
                if (maxVal > max_val_optimal) 
                {
                    max_val_optimal = maxVal;
                    optimal_scale_ = scale_par_;
                }
            }
        }
        // ROS_INFO("Optimal scale found: %f", optimal_scale_);
        // keep scale rate in a threshold to avoid noise
        if ((abs(optimal_scale_ - last_optimal_scale_) >= change_thres_) && (last_optimal_scale_ > 0.0))
        {
            ROS_INFO("Adjust optimal scale");
            if(optimal_scale_ > last_optimal_scale_) optimal_scale_ = last_optimal_scale_ + scale_change_rate_;
            else optimal_scale_ = last_optimal_scale_ - scale_change_rate_;
        }

        // ROS_INFO("Optimal scale: %f", optimal_scale_);
        // ROS_INFO(" Last optimal scale: %f", last_optimal_scale_);

        cv::resize(templ, templ, cv::Size(templ.cols*optimal_scale_, 
                templ.rows*optimal_scale_), optimal_scale_, optimal_scale_);

        last_optimal_scale_ = optimal_scale_;
        if((templ.cols >= img.cols) || (templ.rows >= img.rows))
        {
            ROS_INFO("Template is bigger than image");
            return 0;
        }
        else
        {
            cv::matchTemplate(img, templ, result, match_method_);
            cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
            double threshold = thres_;
            ROS_INFO("Template matching score: %f", maxVal);
            if (maxVal > threshold)
            {
                if (match_method_  == 0)
                {
                    match_method_ = cv::TM_SQDIFF;
                    matchLoc = minLoc;
                } 
                if (match_method_  == 1)
                {
                    match_method_ == cv::TM_SQDIFF_NORMED;
                    matchLoc = minLoc;
                }
                if (match_method_  == 2)
                {
                    match_method_ == cv::TM_CCORR;
                    matchLoc = maxLoc;
                }
                if (match_method_  == 3)
                {
                    match_method_ == cv::TM_CCORR_NORMED;
                    matchLoc = maxLoc;
                }
                if (match_method_  == 4)
                {
                    match_method_ == cv::TM_CCOEFF;
                    matchLoc = maxLoc;
                }

                ROS_INFO("Optimal scale: %f", optimal_scale_);

                // ROS_INFO("minVal: %f", minVal);
                double pose_x_local = double (matchLoc.x) + double (templ.cols)/2;
                double pose_y_local = double (matchLoc.y) + double (templ.rows)/2;

                double center_x = double(img.cols)/2;
                double center_y = double(img.rows)/2;

                double pose_x_camera_frame = ((pose_x_local - center_x)/optimal_scale_)/100;
                double pose_y_camera_frame = ((pose_y_local - center_y)/optimal_scale_)/100;

                pallet_xy.x = pose_x_camera_frame;
                pallet_xy.y = pose_y_camera_frame;

                // std::cout << "pose camera x, y: " << pose_x_camera_frame << ", " << pose_y_camera_frame << std::endl;

                cv::cvtColor(img_display, output, cv::COLOR_GRAY2BGR);
                cv::rectangle( output, matchLoc, 
                    cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), 
                    cv::Scalar(255, 0, 0), 2, 8, 0 );

                if(opencv_show_)
                {
                    // cv::imshow( "Segment Image", img);
                    if(template_matching_show_)
                    {
                        cv::imshow( "Template Matching", output);
                        // cv::imshow( "Template image", templ);
                    }
                    cv::waitKey(1);
                }
                else cv::destroyAllWindows();
                max_point = maxVal;
                return 1;
            }
            else 
            {
                ROS_INFO("Template matching not enough quality");
                return 0;
            }
        }
    }
    
    void kalmanFilter(Eigen::Vector3d &est_pose, Eigen::Matrix3d &est_cov,
                    const Eigen::Vector3d &measure_pose, const Eigen::Matrix3d &measure_cov, bool measurement_avai)
    {
    //kalman update on pose vectors and covariance
        Eigen::Vector3d predict_state;
        Eigen::Matrix3d predict_cov, Rk, Sk, Kk, Ik, Sk_inv;
        Ik.setIdentity();
        Eigen::Vector3d yk;
        Eigen::Vector3d relative_pose;
        
        bool wtf;
        double det;

        // predict model
        predict_state = 1*est_pose;
        predict_cov = est_cov + process_noise_*Eigen::Matrix3d::Identity();
        
        if (measurement_avai)
        {
            relative_pose = measure_pose  - est_pose;
        
            yk = relative_pose;
            
            Rk = measure_cov;
            Sk = predict_cov + Rk;
            Sk.computeInverseAndDetWithCheck(Sk_inv, det, wtf);
            if (!wtf) 
            {
                ROS_WARN("Matrix not invertible");
                return;
            }
            Kk = predict_cov*Sk_inv;

            est_pose = est_pose + Kk*yk;
            est_cov = (Ik - Kk)*est_cov;
        }
        else
        {
            est_pose = predict_state;
            est_cov = predict_cov;
        }

        // std::cout << "kalman filter output: " << std::endl << predict_state << std::endl;
        // std::cout << "estimate covariance: " << std::endl << est_pose << std::endl;
        // std::cout << " covariance: " << std::endl << predict_cov << std::endl;
    }
    
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {   
        pcl::fromROSMsg(*msg, cloud_);  
        ros::Time begin = ros::Time::now();

        /* Filter pointcloud */
        filterPointCloud(cloud_, cloud_f_);
        cloud_ = cloud_f_;
        pub_filtered_pointcloud_.publish(cloud_);

        /* Clear the PC from floor points */
        groundFilter(cloud_, cloud_f_);
        cloud_ = cloud_f_;
        pub_ground_filtered_pointcloud_.publish(cloud_);

        /* Cut off the fork points*/
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_.makeShared());
        pass.setFilterFieldName ("z");
        pass.setFilterLimits(1.0, 3.5);
        pass.setKeepOrganized(true);
        pass.filter (cloud_f_); 
        cloud_ = cloud_f_;

        /* Calculate distance and pallet orientation*/
        pcl_PointCloud::Ptr min_z_points(new pcl_PointCloud);
        min_z_points->header = cloud_.header;
        double max_value_of_min_z_point = 0;
        for (int i = 0; i < cloud_.width; i++)
        {
            double min_z = 5;           //initial meter
            int index = 0;
            pcl::PointXYZ point_min_z;
            for (int j = 0; j < cloud_.height; j++)
            {
                if (isnan(cloud_.at(i,j).x) || isnan(cloud_.at(i,j).y) || isnan(cloud_.at(i,j).z))
                    continue;
                if (cloud_.at(i,j).z < min_z)
                {
                    min_z = cloud_.at(i,j).z;
                    index = j;
                } 
            }
            if (isnan(cloud_.at(i,index).x) || isnan(cloud_.at(i,index).y) || isnan(cloud_.at(i,index).z))
                continue;
            point_min_z.x = cloud_.at(i,index).x;
            point_min_z.y = cloud_.at(i,index).y;
            point_min_z.z = cloud_.at(i,index).z;
            min_z_points->push_back(point_min_z);
        } 
        pub_rgb_pointcloud.publish(min_z_points);

        if (min_z_points->size() != 0)
        {
            /* get Z position of the pallet, z calculate from depth of cluster*/
            pcl::CentroidPoint<pcl::PointXYZ> center_point;

            for (int i = 0; i < min_z_points->size(); i ++)
            {
                if (max_value_of_min_z_point < min_z_points->at(i).z) max_value_of_min_z_point = min_z_points->at(i).z;
                center_point.add(min_z_points->at(i));
            }

            pcl::PointXYZ center_pointxyz;
            center_point.get(center_pointxyz);

            pallet_pose_.header.frame_id = msg->header.frame_id;
            pallet_pose_.pose.position.z = center_pointxyz.z;

            /* get yaw angle of the pallet */
            double vector_pallet_x, vector_pallet_z;
            int index0 = min_z_points->size()/2 - min_z_points->size()/4 ;
            int index1 = min_z_points->size()/2 + min_z_points->size()/4 ;
            vector_pallet_x = min_z_points->at(index1).x - min_z_points->at(index0).x;
            vector_pallet_z = min_z_points->at(index1).z - min_z_points->at(index0).z;
            // vector_pallet_x = min_z_points->at(min_z_points->size()-1).x - min_z_points->at(0).x;
            // vector_pallet_z = min_z_points->at(min_z_points->size()-1).z - min_z_points->at(0).z;

            double vector_length = sqrt(vector_pallet_x*vector_pallet_x + vector_pallet_z*vector_pallet_z);
            pallet_yaw_angle_ = acos(vector_pallet_z/vector_length);
            pallet_yaw_angle_ = pallet_yaw_angle_ - M_PI;
            // ROS_INFO("pallet angle: %f", pallet_yaw_angle_*180/M_PI);
            tf2::Quaternion pallet_quaternion;
            pallet_quaternion.setRPY(0, pallet_yaw_angle_, 0);
            pallet_quaternion = pallet_quaternion.normalize();

            geometry_msgs::Quaternion pallet_final_quaternion;
            tf2::convert(pallet_quaternion, pallet_final_quaternion);
            pallet_pose_.pose.orientation = pallet_final_quaternion;

            pub_pallet_pointcloud_.publish(cloud_);

            bool check_segment = 0;
            if (active_segment_)
            {
                // /* Plane Segment */    
                pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
                estimateNormalKdtree(cloud_.makeShared(), normals);
                check_segment = planeSegementation(cloud_, cloud_rgb_, normals);
                pub_rgb_pointcloud.publish(cloud_rgb_);
            }
            else
            {   // Take all the point to process instead of plane segmentation
                cloud_rgb_.width = cloud_.width;
                cloud_rgb_.height = cloud_.height;
                cloud_rgb_.points.resize(cloud_rgb_.width * cloud_rgb_.height);
                cloud_rgb_.header = cloud_.header;
                for( int i = 0; i < cloud_rgb_.width; i++ ) 
                {
                    for( int j = 0; j < cloud_rgb_.height; j++ ) 
                    {
                        pcl::PointXYZRGB point;
                        point.x = cloud_.at(i,j).x;
                        point.y = cloud_.at(i,j).y;
                        point.z = cloud_.at(i,j).z;
                        if (isnan(point.x) || isnan(point.y) || isnan(point.z)) 
                        {
                            point.r = 0; point.g = 0; point.b = 0;
                        }
                        else 
                        {
                            point.r = 255; point.g = 255; point.b = 255;
                        }
                        cloud_rgb_.at(i,j) = point;
                    }
                }
                check_segment = 1;
            }
            if (check_segment)
            {
                if(cloud_rgb_.size() != 0)
                {       
                    // Convert to opencv image
                    cv::Mat cv_image;
                    cv_image = cv::Mat(cloud_rgb_.height, cloud_rgb_.width, CV_8UC3);
                    
                    for( int y = 0; y < cv_image.rows; y++ ) 
                    {
                        for( int x = 0; x < cv_image.cols; x++ ) 
                        {
                            pcl::PointXYZRGB point = cloud_rgb_.at(x,y );

                            cv_image.at<cv::Vec3b>( y, x )[0] = point.b;
                            cv_image.at<cv::Vec3b>( y, x )[1] = point.g;
                            cv_image.at<cv::Vec3b>( y, x )[2] = point.r;
                        }
                    }

                    cv_image = cv_image - cv::Scalar(0,0,255);
                    cv::Mat img_grey;
                    cv::cvtColor(cv_image, img_grey, CV_BGR2GRAY);
                    cv::Mat img_bw;
                    cv::threshold(img_grey, img_bw, 2.0, 255.0, CV_THRESH_BINARY);
                    source_img_ = img_bw;

                    // /* Template matching */
                    double matching_score;
                    
                    tm_quality_ = templateMatchingPallet(source_img_, templ_img_, result_img_, matching_score, pallet_point_xy_);
                    // ROS_INFO("Matching score: %f", matching_score);
                    

                    if (tm_quality_)
                    {
                        pallet_pose_.pose.position.x = pallet_point_xy_.x;
                        pallet_pose_.pose.position.y = pallet_point_xy_.y;
                        pallet_pose_.pose.position.z += 0.5; 

                        // Transform to base_link
                        pallet_pose_.header.frame_id = msg->header.frame_id;
                        pallet_pose_.header.stamp = ros::Time(0);;

                        tf2_ros::Buffer tf_buffer;
                        tf2_ros::TransformListener listener(tf_buffer);

                        std::string target_link = "odom";

                        try
                        {
                            pallet_pose_output_ = tf_buffer.transform(pallet_pose_, target_link, ros::Duration(1));
                        }
                        catch (tf::LookupException ex)
                        {
                            ROS_ERROR("%s",ex.what());
                        }
                        
                        pub_pallet_pose_.publish(pallet_pose_output_);
                    }
                    else ROS_INFO("Pallet pose is noisy");

                    Eigen::Vector3d pose_measured;
                    Eigen::Matrix3d cov_measured; 
                    cov_measured.setIdentity();
                    cov_measured = 0.015*cov_measured;

                    tf::Quaternion q(
                        pallet_pose_output_.pose.orientation.x,
                        pallet_pose_output_.pose.orientation.y,
                        pallet_pose_output_.pose.orientation.z,
                        pallet_pose_output_.pose.orientation.w);
                    tf::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);

                    pose_measured.x() = pallet_pose_output_.pose.position.x;
                    pose_measured.y() = pallet_pose_output_.pose.position.y; // take z for cal
                    pose_measured.z() = yaw;
                
                    kalmanFilter(tracking_pose_, tracking_cov_, pose_measured, cov_measured, tm_quality_);

                    geometry_msgs::PoseWithCovarianceStamped pallet_pose_kalman;
                    pallet_pose_kalman.header = pallet_pose_output_.header;
                    pallet_pose_kalman.pose.pose.position.x = tracking_pose_.x();
                    pallet_pose_kalman.pose.pose.position.y = tracking_pose_.y();
                    pallet_pose_kalman.pose.pose.position.z = pallet_pose_output_.pose.position.z;

                    tf2::Quaternion pallet_quaternion_kalman;
                    pallet_quaternion_kalman.setRPY(0, tracking_pose_.z(), 0);
                    pallet_quaternion_kalman = pallet_quaternion_kalman.normalize();

                    geometry_msgs::Quaternion pallet_quaternion_kalman_geo;
                    tf2::convert(pallet_quaternion_kalman, pallet_quaternion_kalman_geo);
                    pallet_pose_kalman.pose.pose.orientation = pallet_quaternion_kalman_geo;
                    
                    pallet_pose_kalman.pose.covariance.at(0) = tracking_cov_(0,0);
                    pallet_pose_kalman.pose.covariance.at(7) = 0.1;
                    pallet_pose_kalman.pose.covariance.at(14) = tracking_cov_(1,1);
                    pallet_pose_kalman.pose.covariance.at(21) = 0;
                    pallet_pose_kalman.pose.covariance.at(28) = tracking_cov_(2,2);
                    pallet_pose_kalman.pose.covariance.at(35) = 0;

                    pub_pallet_pose_kalman_.publish(pallet_pose_kalman);

                }
                else ROS_INFO("Segment size = 0!");
            }
            else ROS_INFO("No segment!");
        }
        else ROS_INFO("Non Target");

        pallet_pose_gt_.header.frame_id = "odom";
        pallet_pose_gt_.header.seq = msg->header.seq;
        pallet_pose_gt_.header.stamp = msg->header.stamp;

        pallet_pose_gt_.pose.position.x = pallet_gt_x_;
        pallet_pose_gt_.pose.position.y = pallet_gt_y_;
        pallet_pose_gt_.pose.position.z = 0;

        pub_pallet_gt_.publish(pallet_pose_gt_);

        
        ros::Time end = ros::Time::now();
        ros::Duration processing_time = end - begin;
        ROS_INFO("Processing time: %f \r\n", processing_time.toSec());
    }

    void reconfigCallback(multi_pallet_detection::palletDetectionReconfigureConfig &config, uint32_t level) 
    {
        if (init_reconfig_ == true)
        {
            ROS_INFO("Get init param from launch file");
            init_reconfig_ = false;
        }
        else
        {
            ROS_INFO("Reconfigure Request");
            // Cutoff parameters
            min_x_cutoff_ = config.min_x_cutoff; max_x_cutoff_ = config.max_x_cutoff;
            min_y_cutoff_ = config.min_y_cutoff; max_y_cutoff_ = config.max_y_cutoff;
            min_z_cutoff_ = config.min_z_cutoff; max_z_cutoff_ = config.max_z_cutoff;
            downsample_scale_ = config.downsample_scale;

            // Floor Remove
            active_remove_floor_ = config.active_remove_floor;
            floor_distance_thresh_ = config. floor_distance_thresh;

            // Segment
            active_segment_ = config.active_segment;
            opencv_show_ = config.opencv_show;
            min_cluster_size_ = config. min_cluster_size;
            max_cluster_size_ = config.max_cluster_size;
            neighbor_number_ = config.neighbor_number;
            curvature_thres_ = config.curvature_thres;
            smooth_thresh_ = config.smooth_thresh;
            k_search_ = config.k_search;

            //template matching
            template_matching_show_ = config.template_matching_show;
            match_method_ = config.match_method;
            scale_par_ = config.scale_par;
            manual_scale_ = config.manual_scale;
            min_scale_ = config.min_scale;
            max_scale_ = config.max_scale;
            thres_ = config.thres;
            change_thres_ = config.change_thres;
            scale_change_rate_ = config.scale_change_rate;

            //kalman filter
            process_noise_ = config.process_noise;
            measure_noise_ = config.measure_noise;
        }    
    }

    ~PalletDetection() {}
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pallet_detection");
    ros::NodeHandle param ("~");

    PalletDetection pallet_detection(param);
    ros::spin();
    
    return 0;
}