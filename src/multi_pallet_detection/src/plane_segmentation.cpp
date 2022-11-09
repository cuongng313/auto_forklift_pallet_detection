#include "plane_segmentation.h"

void PlaneSegmentation::preProcessing()
{
    points_.clear();
    points_.resize(point_cloud_->size(), Eigen::Vector3d::Zero());
    double threshold = 20 * 20;
    for (int i = 0; i < point_cloud_->size(); i++)
    {
        // if (isnan(point_cloud_->points[i].x)  ||
        //     isnan(point_cloud_->points[i].y)  ||
        //     isnan(point_cloud_->points[i].z))
        // {
        //     point_cloud_->points[i].x = 0;
        //     point_cloud_->points[i].y = 0;
        //     point_cloud_->points[i].z = 0;
        //     continue;
        // }

        // if(point_cloud_->points[i].z > parameters_.limit_dis)
        //     continue;
            
        // if (point_cloud_->points[i].z < 0 &&
        //     (point_cloud_->points[i].x * point_cloud_->points[i].x +
        //      point_cloud_->points[i].y * point_cloud_->points[i].y) < 0.7225)
        //     continue;
        // if (point_cloud_->points[i].x * point_cloud_->points[i].x +
        //         point_cloud_->points[i].y * point_cloud_->points[i].y +
        //         point_cloud_->points[i].z * point_cloud_->points[i].z >
        //     threshold)
        //     continue;
        if (point_cloud_->points[i].x != 0.0 || point_cloud_->points[i].y != 0.0 || point_cloud_->points[i].z != 0.0)
        {
            points_[i] = Eigen::Vector3d(point_cloud_->points[i].x, point_cloud_->points[i].y, point_cloud_->points[i].z);
        }
    }
}

void PlaneSegmentation::subwindows(int side_length)
{
    // initialize subwindows
    subwindows_.clear();

    // get number subwindow
    subwindows_height_ = static_cast<int>(point_cloud_->height / side_length);
    subwindows_width_ = static_cast<int>(point_cloud_->width / side_length);
    subwindows_.resize(subwindows_height_ * subwindows_width_);
    isPlanar_ = new bool[subwindows_.size()];

    memset(isPlanar_, false, subwindows_.size() * sizeof(bool));

    Eigen::Vector3d sum = Eigen::Vector3d::Zero(), mass_center = Eigen::Vector3d::Zero(), normal = Eigen::Vector3d::Zero();
    Eigen::Matrix3d scatter_matrix = Eigen::Matrix3d::Zero(), second_moment = Eigen::Matrix3d::Zero();
    double bias = 0;

    Eigen::EigenSolver<Eigen::Matrix3d> eigensolver;
    Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
    Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Zero();

    int valid_cnt = 0;
    int row = 0, col = 0;
    int subwindow_size = side_length * side_length;
    valid_indices_ = new int[subwindows_.size() * subwindow_size];
    int pos = 0;
    int planar_cnt = 0;
    std::vector<Subwindow>::iterator it = subwindows_.begin();
    int subwindow_index = 0;
    int *pbegin = NULL;
    int *pend = NULL;
    int *p = NULL;

    // std::cout << "subwindows_height_: " << subwindows_height_ << std::endl
    //             << "subwindows_width_" << subwindows_width_ << std::endl;

    // std::cout << "height_ - side_length: " << height_ - side_length << std::endl
    //             << "width_ - side_length: " << width_ - side_length << std::endl;

    for (int i = 0; i < height_ - side_length; i += side_length)
    {
        for (int j = 0; j < width_ - side_length; j += side_length)
        {
            pbegin = valid_indices_ + subwindow_index * subwindow_size;

            sum = Eigen::Vector3d::Zero();
            scatter_matrix = Eigen::Matrix3d::Zero();
            second_moment = Eigen::Matrix3d::Zero();
            valid_cnt = 0;

            p = pbegin;
            for (row = i; row < i + side_length; row++)
            {
                for (col = j; col < j + side_length; col++)
                {
                    pos = row * width_ + col;           // calculate the 1-dimension position
                    if (!valid_[pos])
                        continue;
                    valid_cnt++;                    // count number valid point
                    *(p++) = pos;
                }
            }
            pend = p;
            it->point_num = valid_cnt;                // update number valid point in the considered subwindow
            
            if (valid_cnt < static_cast<int>(subwindow_size * 0.7))     //////// spare type
            {
                it++;
                subwindow_index++;
                continue;
            }
            for (p = pbegin; p < pend; p++)
            {
                sum += points_[*p];
                second_moment += points_[*p] * points_[*p].transpose();
            }
            mass_center = sum / static_cast<double>(valid_cnt);
            for (p = pbegin; p < pend; p++)
            {
                scatter_matrix += (points_[*p] - mass_center) * (points_[*p] - mass_center).transpose();
            }
            eigensolver.compute(scatter_matrix);
            eigenvalues = eigensolver.eigenvalues().real();
            eigenvectors = eigensolver.eigenvectors().real();

            int min_eigenvalue_index;
            double lambda[2];
            double min_eigenvalue = eigenvalues.minCoeff(&min_eigenvalue_index);
            int lambda_index = 0;
            for (int q = 0; q < 3; q++)
            {
                if (q == min_eigenvalue_index)
                    continue;
                lambda[lambda_index] = eigenvalues(q) / min_eigenvalue;
                lambda_index++;
            }
            double min_lambda = std::min(lambda[0], lambda[1]);

            // non-planar
            if (min_lambda < 2.0 * side_length)
            {
                it++;
                subwindow_index++;
                continue;
            }

            //planar windows
            it->normal = eigenvectors.col(min_eigenvalue_index); // normal vector of planar
            it->bias = it->normal.dot(mass_center);
            if (it->bias < 0)
            {
                it->normal = -it->normal;
                it->bias = -it->bias;
            }
            it->sum = sum;
            it->mass_center = mass_center;
            it->second_moment = second_moment;
            it->mse = min_eigenvalue / static_cast<double>(valid_cnt);
            isPlanar_[subwindow_index] = true;
            planar_cnt++;
            it++;
            subwindow_index++;
        }
    }

    planar_subwindows_cnt_ = planar_cnt;
    sorted_subwindows_.resize(planar_subwindows_cnt_);
    subwindow_index = 0;
    for (int i = 0; i < subwindows_.size(); i++)
    {
        if (isPlanar_[i])
        {
            sorted_subwindows_[subwindow_index].index = i;
            sorted_subwindows_[subwindow_index].mse = subwindows_[i].mse;
            subwindow_index++;
        }
    }
    //  int tmp = -1;
    //  for (int i = planar_subwindows_cnt_ - 1; i > 0; i--)
    //  {
    //    for(int j = 0; j < i; j++)
    //    {
    //      if(subwindows_[sorted_subwindows_[i]].mse < subwindows_[sorted_subwindows_[j]].mse)
    //      {
    //        tmp = sorted_subwindows_[i];
    //        sorted_subwindows_[i] = sorted_subwindows_[j];
    //        sorted_subwindows_[j] = tmp;
    //      }
    //    }
    //  }
    std::sort(sorted_subwindows_.begin(), sorted_subwindows_.end());
    ROS_INFO("Done for plane classification");
}

void PlaneSegmentation::investigateNeighbors(const int index)
{
    // index: position of the considered subwindow
    int row = index / subwindows_width_;
    int col = index % subwindows_width_;

    // std::cout << "index: " << index << std::endl;
    // std::cout << "subwindows_width_: " << subwindows_width_ << std::endl;
    // std::cout << "row: " << row << std::endl;
    // std::cout << "col: " << col << std::endl;
    for (int i = -1; i < 2; i++)
    {
        for (int j = -1; j < 2; j++)
        {
            if ((i || j) 
                && row + i >= 0 && row + i < subwindows_height_ 
                && col + j >= 0 && col + j < subwindows_width_)
            {
                int pos = (row + i) * subwindows_width_ + (col + j);
                if (!visited_[pos] && !added_to_region_[pos])
                {
                    neighbors_.push_back(pos);
                    visited_[pos] = true;
                }
            }
        }
    }
}

void PlaneSegmentation::applySegmentation()
{
    width_ = point_cloud_->width;
    height_ = point_cloud_->height;
    size_ = height_ * width_;
    valid_ = new bool[size_];

    // std::cout << " w, h, s, v: " << width_ << " "
    //                             << height_ << " "
    //                             << size_ << " " << std::endl;

    memset(valid_, false, size_ * sizeof(bool));
    valid_points_ = 0;
    for (int i = 0; i < size_; i++)
    {
        if (points_[i](0) != 0.0 || points_[i](1) != 0.0 || points_[i](2) != 0.0)
        {
            valid_[i] = true;
            valid_points_++;
        }
    }
    std::cout << "solve valid point" << std::endl;

    subwindows(parameters_.subwindow_side_length);

    std::cout << "planar_subwindows_cnt_: " << planar_subwindows_cnt_ << std::endl;

    int subwindow_size = parameters_.subwindow_side_length * parameters_.subwindow_side_length;
    planar_patches_.clear();
    remained_points_.clear();
    added_to_region_ = new bool[subwindows_.size()];
    memset(added_to_region_, false, subwindows_.size() * sizeof(bool));
    visited_ = new bool[subwindows_.size()];

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d mass_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Matrix3d second_moment = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d scatter_matrix = Eigen::Matrix3d::Zero();
    int point_num = 0;
    double bias;

    Eigen::EigenSolver<Eigen::Matrix3d> eigensolver;
    Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
    Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Zero();

    int index = 0;
    int pos = 0;
    int planar_index = 0;

    while (index < planar_subwindows_cnt_)
    {   
        // std::cout << "index: " << index << std::endl;
        // std::cout << "planar subwindow count: " << planar_subwindows_cnt_ << std::endl;
        while (index < planar_subwindows_cnt_ - 1 && added_to_region_[sorted_subwindows_[index].index])
        {
            index++;
        }
        // std::cout << "check here" << std::endl;
        // std::cout << "index after check: " << index << std::endl;
        // std::cout << "subwindows_[sorted_subwindows_[index].index].mse: "
        //         << subwindows_[sorted_subwindows_[index].index].mse << std::endl;
        // std::cout << "parameters_.max_segment_mse: "
                    // << parameters_.max_segment_mse << std::endl;
        if (index == planar_subwindows_cnt_ - 1 && subwindows_[sorted_subwindows_[index].index].mse >= parameters_.max_segment_mse)
            break;

        memset(visited_, false, subwindows_.size() * sizeof(bool));
        PlanarSegment tmp_pp;
        neighbors_.clear();
        pos = sorted_subwindows_[index].index;
        tmp_pp.sum = subwindows_[pos].sum;
        tmp_pp.mass_center = subwindows_[pos].mass_center;
        tmp_pp.normal = subwindows_[pos].normal;
        tmp_pp.second_moment = subwindows_[pos].second_moment;
        tmp_pp.bias = subwindows_[pos].bias;
        tmp_pp.mse = subwindows_[pos].mse;
        tmp_pp.point_num = subwindows_[pos].point_num;
        tmp_pp.points.resize(valid_points_);

        // std::cout << "index after cal: " << index << std::endl;
        // std::cout << "pos: " << pos << std::endl;
        for (int i = 0, *p = valid_indices_ + pos * subwindow_size; i < tmp_pp.point_num; i++, p++)
        {
            tmp_pp.points[i] = *p;
        }
        investigateNeighbors(pos);
        int cnt = 0;
        for (int i = 0; i < neighbors_.size(); i++)
        {
            if (isPlanar_[neighbors_[i]] && subwindows_[neighbors_[i]].normal.dot(tmp_pp.normal) > parameters_.min_dot_product)
                cnt++;
        }
        if (cnt < parameters_.valid_neighbor_cnt)
        {
            index++;
            continue;
        }
        while (!neighbors_.empty())
        {
            pos = neighbors_.front();
            neighbors_.erase(neighbors_.begin());
            if (isPlanar_[pos])
            {
                if (subwindows_[pos].normal.dot(tmp_pp.normal) < parameters_.min_dot_product)
                {
                    visited_[pos] = false;
                    continue;
                }
                if (fabs(tmp_pp.normal.dot(subwindows_[pos].mass_center) - tmp_pp.bias) > parameters_.max_mass2plane_dis)
                {
                    visited_[pos] = false;
                    continue;
                }
                point_num = tmp_pp.point_num + subwindows_[pos].point_num;
                sum = tmp_pp.sum + subwindows_[pos].sum;
                mass_center = sum / static_cast<double>(point_num);
                second_moment = tmp_pp.second_moment + subwindows_[pos].second_moment;
                scatter_matrix = second_moment - sum * mass_center.transpose();
                
                eigensolver.compute(scatter_matrix);
                eigenvalues = eigensolver.eigenvalues().real();
                eigenvectors = eigensolver.eigenvectors().real();
                int min_eigenvalue_index;
                double min_eigenvalue = eigenvalues.minCoeff(&min_eigenvalue_index);
                if (min_eigenvalue / static_cast<double>(point_num) > parameters_.max_segment_mse)
                {
                    visited_[pos] = false;
                    continue;
                }
                normal = eigenvectors.col(min_eigenvalue_index);
                bias = normal.dot(mass_center);
                if (bias < 0)
                {
                    normal = -normal;
                    bias = -bias;
                }
                tmp_pp.sum = sum;
                tmp_pp.mass_center = mass_center;
                tmp_pp.second_moment = second_moment;
                tmp_pp.normal = normal;
                tmp_pp.bias = bias;
                for (int i = tmp_pp.point_num, *p = valid_indices_ + pos * subwindow_size; i < point_num; i++, p++)
                {
                    tmp_pp.points[i] = *p;
                }
                tmp_pp.point_num = point_num;
                added_to_region_[pos] = true;
                investigateNeighbors(pos);
            }
            else
            {
                int added_points = 0;
                for (int *p = valid_indices_ + pos * subwindow_size;
                     p != valid_indices_ + pos * subwindow_size + subwindows_[pos].point_num; p++)
                {
                    Eigen::Vector3d point = points_[*p];
                    point_num = tmp_pp.point_num + 1;
                    sum = tmp_pp.sum + point;
                    mass_center = sum / static_cast<double>(point_num);
                    second_moment = tmp_pp.second_moment + point * point.transpose();
                    scatter_matrix = second_moment - sum * mass_center.transpose();
                    eigensolver.compute(scatter_matrix);
                    eigenvalues = eigensolver.eigenvalues().real();
                    eigenvectors = eigensolver.eigenvectors().real();
                    int min_eigenvalue_index;
                    double min_eigenvalue = eigenvalues.minCoeff(&min_eigenvalue_index);
                    if (min_eigenvalue / static_cast<double>(point_num) > parameters_.max_segment_mse)
                        continue;
                    normal = eigenvectors.col(min_eigenvalue_index);
                    bias = normal.dot(mass_center);
                    if (bias < 0)
                    {
                        normal = -normal;
                        bias = -bias;
                    }
                    if (fabs(normal.dot(point - mass_center)) > parameters_.max_mass2plane_dis)
                        continue;
                    tmp_pp.points[point_num - 1] = *p;
                    tmp_pp.point_num = point_num;
                    tmp_pp.sum = sum;
                    tmp_pp.mass_center = mass_center;
                    tmp_pp.second_moment = second_moment;
                    tmp_pp.normal = normal;
                    tmp_pp.bias = bias;
                    added_points++;
                }
                visited_[pos] = true;
                if (static_cast<double>(added_points) / static_cast<double>(subwindows_[pos].point_num) > 0.6)
                {
                    added_to_region_[pos] = true;
                    investigateNeighbors(pos);
                }
            }
        }
        tmp_pp.points.erase(tmp_pp.points.begin() + tmp_pp.point_num, tmp_pp.points.end());
        
        if (tmp_pp.point_num > parameters_.min_segment_size)
        {
            planar_patches_.push_back(tmp_pp);
            std::cout << "planar plane [" << planar_index << "]" << std::endl;
            std::cout << "plane center (x,y,z): " << tmp_pp.mass_center.x() << "  "
                                                 << tmp_pp.mass_center.y()  << "  "
                                                << tmp_pp.mass_center.z() << std::endl;
            std::cout << "plane normal vector: " << tmp_pp.normal.x() << "  "
                                                << tmp_pp.normal.y()  << "  "
                                                << tmp_pp.normal.z() << std::endl;
            std::cout << "number point: " << tmp_pp.point_num << std::endl << std::endl;
            planar_index++;
        }
        else
        {}
        // std::cout << "index after cal end: " << index << std::endl;
        index++;
        
    }
    std::cout << "plane segment done" << std::endl;
    process_done = 1;
}





void PlaneSegmentation::colorEncoding (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output, bool project2plane)
{
  output->points.clear();
  output->points.resize(point_cloud_->size());

  srand ( time(NULL) );
  int cnt = 0;
  for (PlanarSegment::StdVector::iterator it = planar_patches_.begin(); it != planar_patches_.end(); it++)
  {
    double rgb = rand();
    if (!project2plane)
    {
      for (int j = 0; j < it->points.size(); j++)
      {
        int index = it->points[j];
        output->points[cnt].x = point_cloud_->points[index].x;
        output->points[cnt].y = point_cloud_->points[index].y;
        output->points[cnt].z = point_cloud_->points[index].z;
        output->points[cnt].rgb = rgb;
        // std::cout << "color: " << rgb << std::endl;
        cnt ++;
      }
    }
    else
    {
      for (int j = 0; j < it->points.size(); j++)
      {
        int index = it->points[j];
        double dis = point_cloud_->points[index].x * it->normal(0) +
                    point_cloud_->points[index].y * it->normal(1) +
                    point_cloud_->points[index].z * it->normal(2) - it->bias;
        output->points[cnt].x = point_cloud_->points[index].x - dis * it->normal(0);
        output->points[cnt].y = point_cloud_->points[index].y - dis * it->normal(1);
        output->points[cnt].z = point_cloud_->points[index].z - dis * it->normal(2);
        output->points[cnt].rgb = rgb;
        cnt ++;
      }
    }
  }
  output->points.erase(output->points.begin() + cnt, output->points.end());
  output->height = 1;
  output->width = cnt;
  output->resize(cnt);
}