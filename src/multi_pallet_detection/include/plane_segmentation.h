#include <ros/ros.h>
#include <Eigen/Core>
// PCL library header
#include <pcl_ros/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/visualization/point_cloud_handlers.h"
#include <pcl/filters/voxel_grid.h>

#include <math.h>

struct SimpleSubwindow
{
  int index;
  double mse;
  SimpleSubwindow () :
    index (0), mse (0)
  {}
  ~SimpleSubwindow ()
  {}
  bool
  operator< (const SimpleSubwindow& rhs) const
  {
    return mse < rhs.mse;
  }
};

struct Subwindow
{
  typedef boost::shared_ptr<Subwindow> Ptr;
  typedef std::vector<Subwindow> StdVector;
  typedef boost::shared_ptr<StdVector> StdVectorPtr;
  Eigen::Vector3d sum;
  Eigen::Vector3d mass_center;
  Eigen::Vector3d normal;
  Eigen::Matrix3d second_moment;
  int point_num;
  double bias;
  double mse;
  std::vector<int> points;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Subwindow ():
  sum (Eigen::Vector3d::Zero()), mass_center (Eigen::Vector3d::Zero()), 
  normal (Eigen::Vector3d::Zero()), second_moment (Eigen::Matrix3d::Zero()),
  point_num (0), bias (0.0), mse (0.0) {}

  ~Subwindow () {}
};

struct PlanarSegment
{
    public:
      typedef boost::shared_ptr<PlanarSegment> Ptr;
      typedef std::vector<PlanarSegment, Eigen::aligned_allocator<PlanarSegment> > StdVector;
      typedef boost::shared_ptr<StdVector> StdVectorPtr;
      PlanarSegment () :
      sum (Eigen::Vector3d::Zero ()), mass_center (Eigen::Vector3d::Zero ()), normal (Eigen::Vector3d::Zero ()), second_moment (Eigen::Matrix3d::Zero ()),
      scatter_matrix (Eigen::Matrix3d::Zero ()), Cnn(Eigen::Matrix3d::Zero()), hessian(Eigen::Matrix4d::Zero ()), covariance (Eigen::Matrix4d::Zero()),
      bias (0.0), mse (0.0), area (0.0), Cdd (0.0), Cnn_trace (0.0), Dcovariance (0.0), point_num (0){
      }
      Eigen::Vector3d sum;
      Eigen::Vector3d mass_center;
      Eigen::Vector3d normal;
      Eigen::Matrix3d second_moment;
      Eigen::Matrix3d scatter_matrix;
      Eigen::Matrix3d Cnn;
      Eigen::Matrix4d hessian;
      Eigen::Matrix4d covariance;
      double bias;
      double mse;
      double area;
      double Cdd;
      double Cnn_trace;
      double Dcovariance;
      int point_num;
      std::vector<int> points;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct HybridRGSegmentationParameters
{
    double min_dot_product;
    double max_mass2plane_dis;
    double max_segment_mse;
    int min_segment_size;
    int subwindow_side_length;
    double limit_dis;
    int valid_neighbor_cnt;
    HybridRGSegmentationParameters():
        min_dot_product (0.0), max_mass2plane_dis (0.0), max_segment_mse (0.0), min_segment_size (0),
        subwindow_side_length (0), limit_dis (0.0), valid_neighbor_cnt (0.0)
    {}
};


class PlaneSegmentation 
{
    public:
        PlaneSegmentation(): 
        point_cloud_ (new pcl::PointCloud<pcl::PointXYZ>), size_ (0), height_ (0), width_ (0),
        subwindows_height_ (0), subwindows_width_ (0), valid_ (NULL), visited_ (NULL),
        added_to_region_ (NULL), isPlanar_ (NULL), local_mse_ (NULL), valid_points_ (0),
        planar_subwindows_cnt_ (0), badpoints_num_ (0), valid_indices_ (NULL)
        {}

        void setInput (pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
        {
            point_cloud_ = point_cloud;
        }

        void preProcessing();

        void setParameter(const HybridRGSegmentationParameters parameters)
        {
            parameters_ = parameters;
        }

        void subwindows(int side_length);

        void investigateNeighbors (const int index);

        void applySegmentation();

        void colorEncoding (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output,
                 bool project2plane);


    public:
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points_;
        HybridRGSegmentationParameters parameters_;
        Subwindow::StdVector subwindows_;
        int size_;
        int height_;
        int width_;
        int subwindows_height_;
        int subwindows_width_;
        bool *isPlanar_;
        int *valid_indices_;
        bool *valid_;
        int planar_subwindows_cnt_;
        bool *visited_;
        bool *added_to_region_;
        std::deque<int> neighbors_;

        std::vector<SimpleSubwindow> sorted_subwindows_;
        int valid_points_;
        PlanarSegment::StdVector planar_patches_;
        std::vector<int> remained_points_;

        double *local_mse_;
        int badpoints_num_;
        

        bool process_done = 0;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};