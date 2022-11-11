#include <ros/ros.h>
#include <Eigen/Core>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

/* Plane Segmentation */
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

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

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"

#include <dynamic_reconfigure/server.h>
#include </home/cuongnguen/Techtile/auto_forklift/devel/include/multi_pallet_detection/palletDetectionReconfigureConfig.h>



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

    ros::Publisher pub_pointcloud_normal_;
    ros::Publisher pub_nan_boundary_edges_indices_,
      pub_occluding_edges_indices_, pub_occluded_edges_indices_,
      pub_curvature_edges_indices_, pub_rgb_edges_indices_, pub_all_edges_indices_;
    ros::Publisher pub_nan_boundary_edges_,
      pub_occluding_edges_, pub_occluded_edges_,
      pub_curvature_edges_, pub_rgb_edges_, pub_all_edges_;
    ros::Publisher pub_image_;

    ros::Subscriber sub_point_cloud_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    
    /* Pointcloud cutoff */
    double min_x_cutoff_;
    double max_x_cutoff_;
    double min_y_cutoff_;
    double max_y_cutoff_;
    double min_z_cutoff_;
    double max_z_cutoff_;

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

    /* Edge detection */
    bool visualize_normal_;
    double depth_discontinuation_threshold_;
    int max_search_neighbors_;
    bool use_nan_boundary_;
    bool use_occluding_;
    bool use_occluded_;
    bool use_curvature_;
    bool use_rgb_;

    /* Template matching */
    cv::Mat templ_img;
    cv::Mat source_img_;
    cv::Mat result_img_;
    int match_method_ = 0;

    pcl_PointCloud cloud_;
    pcl_PointCloud cloud_f_;
    pcl_PointCloudRGB cloud_rgb_;
    sensor_msgs::Image pallet_image_;

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

        /* Ground filter */
        paramGet.param<bool>("active_remove_floor", active_remove_floor_, false);
        paramGet.param<double>("floor_distance_thresh", floor_distance_thresh_, 0.01);

        /* Plane segmentation */
        paramGet.param<bool>("active_segment", active_segment_, false);
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

        /* edge detection */
        paramGet.param<double>("depth_discontinuation_threshold", depth_discontinuation_threshold_, 0.04);
        paramGet.param<int>("max_search_neighbors", max_search_neighbors_, 100);
        paramGet.param<bool>("use_nan_boundary", use_nan_boundary_, true);
        paramGet.param<bool>("use_occluding", use_occluding_, true);
        paramGet.param<bool>("use_occluded", use_occluded_, true);
        paramGet.param<bool>("use_curvature", use_curvature_, false);
        paramGet.param<bool>("use_rgb", use_rgb_, false);

        /* template matching */
        paramGet.param<int>("match_method", match_method_, 0);

        /* ROS Subscriber */
        sub_point_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, 
                                                        &PalletDetection::pointCloudCallback, this);

        /* ROS Publisher */
        pub_filtered_pointcloud_ = nh_.advertise<pcl_PointCloudRGB>("pd_filtered_pointcloud", 1);
        pub_ground_filtered_pointcloud_ = nh_.advertise<pcl_PointCloudRGB>("pd_ground_filtered_pointcloud", 1);
        pub_pallet_pointcloud_ = nh_.advertise<pcl_PointCloudRGB>("pd_pallet_pointcloud", 1);
        pub_rgb_pointcloud = nh_.advertise<pcl_PointCloudRGB>("pd_pallet_rgb_pointcloud", 1);

        // Edge indice publisher
        pub_pointcloud_normal_ = nh_.advertise<PointCloudN>("pd_output_point_cloud_normal", 1);
        pub_nan_boundary_edges_indices_ = nh_.advertise<PCLIndicesMsg>("pd_output_nan_boundary_edge_indices", 1);
        pub_occluding_edges_indices_= nh_.advertise<PCLIndicesMsg>("pd_output_occluding_edge_indices", 1);
        pub_occluded_edges_indices_= nh_.advertise<PCLIndicesMsg>("pd_output_occluded_edge_indices", 1);
        pub_curvature_edges_indices_ = nh_.advertise<PCLIndicesMsg>("pd_output_curvature_edge_indices", 1);
        pub_rgb_edges_indices_ = nh_.advertise<PCLIndicesMsg>("pd_output_rgb_edge_indices", 1);
        pub_all_edges_indices_= nh_.advertise<PCLIndicesMsg>("pd_output_indices", 1);

        // Edge publisher
        pub_nan_boundary_edges_= nh_.advertise<sensor_msgs::PointCloud2>("pd_output_nan_boundary_edge", 1);
        pub_occluding_edges_= nh_.advertise<sensor_msgs::PointCloud2>("pd_output_occluding_edge", 1);
        pub_occluded_edges_= nh_.advertise<sensor_msgs::PointCloud2>("pd_output_occluded_edge", 1);
        pub_curvature_edges_= nh_.advertise<sensor_msgs::PointCloud2>("pd_output_curvature_edge", 1);
        pub_rgb_edges_= nh_.advertise<sensor_msgs::PointCloud2>("pd_output_rgb_edge", 1);
        pub_all_edges_= nh_.advertise<sensor_msgs::PointCloud2>("pd_output", 1);

        // Image publisher
        pub_image_= nh_.advertise<sensor_msgs::Image>("pd_image_converted", 1);

        // dynamic reconfigure server

        srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (paramGet);
        dynamic_reconfigure::Server<Config>::CallbackType f;
        f = boost::bind(&PalletDetection::reconfigCallback, this, _1, _2);
        srv_->setCallback(f);
    }

    void filterPointCloud(pcl_PointCloud &input, pcl_PointCloud &output) 
    {
        // Filter cloud
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
    
    void planeSegementation(pcl_PointCloud &input,pcl_PointCloudRGB &output, 
                            const pcl::PointCloud<pcl::Normal>::Ptr& normals)
    {
        // pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
        // estimateNormalKdtree(input.makeShared(), normals);

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
        // std::vector<std::pair<long int, int> > vp;

        // Filter out paller pointcloud
        // for (int i = 0; i < clusters.size(); i++)
        // {
        //     vp.push_back(std::make_pair(clusters[i].indices.size(), i));
        // }
        // std::sort(vp.begin(), vp.end());

        // int min_index = vp[0].second;
        // int second_min_index = vp[1].second;
        
        // for (int i = 0; i < cluster_size; i++)
        // {
        //     std::cout << "Points of cluster [" << i << "]: " << vp[i].first << std::endl;
        // }

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

        }
        else{
            std::cout << "no cluster found!" << std::endl;
        }

    }

    /* Edge detection module */
    void estimateNormalIntegralImage( const pcl_PointCloud::Ptr &input,
                            pcl::PointCloud<pcl::Normal>::Ptr output)
    {
        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        if (normal_estimation_method_ == 0) 
        {
            ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
        }
        else if (normal_estimation_method_ == 1) 
        {
            ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
        }
        else if (normal_estimation_method_ == 2) 
        {
            ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
        }
        else 
        {
            ROS_INFO("unknown estimation method");
            return;
        }

        if (border_policy_ignore_) 
        {
            ne.setBorderPolicy(pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::BORDER_POLICY_IGNORE);
        }
        else 
        {
            ne.setBorderPolicy(pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::BORDER_POLICY_MIRROR);
        }

        ne.setMaxDepthChangeFactor(max_depth_change_factor_);
        ne.setNormalSmoothingSize(normal_smoothing_size_);
        ne.setDepthDependentSmoothing(depth_dependent_smoothing_);

        ne.setInputCloud(input);

        ne.compute(*output);

        if (publish_normal_) 
        {
            pub_pointcloud_normal_.publish(output);
        }
        
        if (visualize_normal_)
        {
            pcl::visualization::PCLVisualizer viewer("Point Normal");
            viewer.setBackgroundColor (0.0, 0.0, 0.5);
            viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(input, output);
            while (!viewer.wasStopped ())
            {
                viewer.spinOnce ();
            }
        }

    }

    void estimateEdge( const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, const pcl::PointCloud<pcl::Normal>::Ptr& normal,
                        pcl::PointCloud<pcl::Label>::Ptr& output, std::vector<pcl::PointIndices>& label_indices)
    {
        pcl::OrganizedEdgeFromNormals<pcl::PointXYZ, pcl::Normal, pcl::Label> oed;
        oed.setDepthDisconThreshold (depth_discontinuation_threshold_);
        oed.setMaxSearchNeighbors (max_search_neighbors_);
        int flags = 0;
        if (use_nan_boundary_) 
        {
            flags |= oed.EDGELABEL_NAN_BOUNDARY;
        }
        if (use_occluding_) {
            flags |= oed.EDGELABEL_OCCLUDING;
        }
        if (use_occluded_) {
            flags |= oed.EDGELABEL_OCCLUDED;
        }
        if (use_curvature_) {
            flags |= oed.EDGELABEL_HIGH_CURVATURE;
        }
        if (use_rgb_) {
            flags |= oed.EDGELABEL_RGB_CANNY;
        }
        oed.setEdgeType (flags);
        oed.setInputNormals(normal);
        oed.setInputCloud(input);
        oed.compute(*output, label_indices);
    }
     
    void publishEdge( ros::Publisher& pub, ros::Publisher& pub_indices,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<int>& indices,
                            const std_msgs::Header& header)
    {
        // publish indices
        pcl_msgs::PointIndices msg;
        msg.header = header;
        msg.indices = indices;
        pub_indices.publish(msg);

        // publish cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, indices, *output);
        sensor_msgs::PointCloud2 ros_output;
        pcl::toROSMsg(*output, ros_output);
        ros_output.header = header;
        pub.publish(ros_output);
    }
    
    /* Template Matching */
    void templateMatchingPallet(cv::Mat img, cv::Mat templ, cv::Mat output)
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

        match_method_ = 0;
        cv::matchTemplate(img, templ, result, match_method_);

        cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
        cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

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

        cv::cvtColor(img_display, output, cv::COLOR_GRAY2BGR);

        cv::rectangle( output, matchLoc, 
            cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), 
            cv::Scalar(255, 0, 0), 2, 8, 0 );
        cv::rectangle( result, matchLoc, 
            cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), 
            cv::Scalar(255, 0, 0), 2, 8, 0 );

        // img_display.copyTo(output);

        if(opencv_show_)
        {
            cv::imshow( "Segment Image", img);
            cv::imshow( "Template Matching", output);
            cv::waitKey(3);
        }
        else 
        {
            cv::destroyAllWindows();
        }

    }
    
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::fromROSMsg(*msg, cloud_);  

        /* Filter pointcloud */
        filterPointCloud(cloud_, cloud_f_);
        cloud_ = cloud_f_;
        pub_filtered_pointcloud_.publish(cloud_);

        /* Clear the PC from floor points */
        if(active_remove_floor_)
        {
            groundFilter(cloud_, cloud_f_);
            cloud_ = cloud_f_;
            pub_ground_filtered_pointcloud_.publish(cloud_);
        }

        /* Plane Segment */    
        if(active_segment_)
        {
            pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
            estimateNormalKdtree(cloud_.makeShared(), normals);
            planeSegementation(cloud_, cloud_rgb_, normals);
            
            pub_rgb_pointcloud.publish(cloud_rgb_);
            // pub_pallet_pointcloud_.publish(cloud_f_);
            try
            {
                pcl::toROSMsg(cloud_rgb_, pallet_image_);
   
                pallet_image_.header = msg->header;   
                // std::cout << "Image frame: " << pallet_image_.header.frame_id << std::endl;
                // std::cout << "Image size h and w: " << pallet_image_.height << " and "
                //                                     << pallet_image_.width << std::endl;
            }
            catch(std::runtime_error e)
            {
                ROS_ERROR_STREAM("Error in converting cloud to image message: "
                                            << e.what());
            }

            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(pallet_image_, sensor_msgs::image_encodings::RGB8);
            cv_ptr->image = cv_ptr->image - cv::Scalar(255,0,0);

            cv::Mat img_grey;
            cv::cvtColor(cv_ptr->image, img_grey, CV_BGR2GRAY);

            cv::Mat img_bw;
            cv::threshold(img_grey, img_bw, 2.0, 255.0, CV_THRESH_BINARY);
            source_img_ = img_bw;
            // std::cout << "opencv image size: " << img_bw.size() << std::endl;

            pub_image_.publish(pallet_image_);
        }

        /* Normal estimation*/
        pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normal(new pcl::PointCloud<pcl::Normal>);
        estimateNormalIntegralImage(cloud_.makeShared(), pointcloud_normal);
        // estimateNormalKdtree(cloud_.makeShared(), pointcloud_normal);

        /* Edge estimation*/
        pcl::PointCloud<pcl::Label>::Ptr label(new pcl::PointCloud<pcl::Label>);
        std::vector<pcl::PointIndices> label_indices;

        estimateEdge(cloud_.makeShared(), pointcloud_normal, label, label_indices);

        publishEdge(pub_nan_boundary_edges_, pub_nan_boundary_edges_indices_,
                    cloud_.makeShared(), label_indices[0].indices, msg->header);
        publishEdge(pub_occluding_edges_, pub_occluding_edges_indices_,
                    cloud_.makeShared(), label_indices[1].indices, msg->header);
        publishEdge(pub_occluded_edges_, pub_occluded_edges_indices_,
                    cloud_.makeShared(), label_indices[2].indices, msg->header);
        publishEdge(pub_curvature_edges_, pub_curvature_edges_indices_,
                    cloud_.makeShared(), label_indices[3].indices, msg->header);
        publishEdge(pub_rgb_edges_, pub_rgb_edges_indices_,
                    cloud_.makeShared(), label_indices[4].indices, msg->header); 


        templ_img = cv::imread("/home/cuongnguen/Techtile/auto_forklift/src/multi_pallet_detection/template/pallet1cell1cm.png", cv::IMREAD_GRAYSCALE);

        templateMatchingPallet(source_img_, templ_img, result_img_);
        std::cout << "output size: " << result_img_.size() << std::endl;


    }

    void reconfigCallback(multi_pallet_detection::palletDetectionReconfigureConfig &config, uint32_t level) {
        ROS_INFO("Reconfigure Request");
        // Cutoff parameters
        min_x_cutoff_ = config.min_x_cutoff; max_x_cutoff_ = config.max_x_cutoff;
        min_y_cutoff_ = config.min_y_cutoff; max_y_cutoff_ = config.max_y_cutoff;
        min_z_cutoff_ = config.min_z_cutoff; max_z_cutoff_ = config.max_z_cutoff;

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

        // Edge Detection
        visualize_normal_ = config.visualize_normal;
        depth_discontinuation_threshold_ = config.depth_discontinuation_threshold;
        max_search_neighbors_ = config.max_search_neighbors;
        use_nan_boundary_ = config.use_nan_boundary;
        use_occluding_ = config.use_occluding;
        use_occluded_ = config.use_occluded;
        use_curvature_ = config.use_curvature;
        use_rgb_ = config.use_rgb;
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