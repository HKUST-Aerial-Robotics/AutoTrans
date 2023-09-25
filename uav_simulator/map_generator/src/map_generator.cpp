#include "map_generator/map_generator.hpp"

namespace map_generator
{
    void MapGenerator::init(ros::NodeHandle &nh)
    {
        nh_ = nh;
        map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

        nh.param("map_generator/map/x_size", generator_params_.x_size_, 50.0);
        nh.param("map_generator/map/y_size", generator_params_.y_size_, 50.0);
        nh.param("map_generator/map/z_size", generator_params_.z_size_, 5.0);
        nh.param("map_generator/map/obs_num", generator_params_.obs_num_, 30);
        nh.param("map_generator/map/circle_num", generator_params_.circle_num_, 30);
        nh.param("map_generator/map/resolution", generator_params_.resolution_, 0.1);
        nh.param("map_generator/map/seed", generator_params_.seed_, 0);

        nh.param("map_generator/obstacle/lower_rad", generator_params_.w_l_, 0.3);
        nh.param("map_generator/obstacle/upper_rad", generator_params_.w_h_, 0.8);
        nh.param("map_generator/obstacle/lower_hei", generator_params_.h_l_, 3.0);
        nh.param("map_generator/obstacle/upper_hei", generator_params_.h_h_, 7.0);

        nh.param("map_generator/obstacle/radius_l", generator_params_.radius_l_, 7.0);
        nh.param("map_generator/obstacle/radius_h", generator_params_.radius_h_, 7.0);
        nh.param("map_generator/obstacle/z_l", generator_params_.z_l_, 7.0);
        nh.param("map_generator/obstacle/z_h", generator_params_.z_h_, 7.0);
        nh.param("map_generator/obstacle/theta", generator_params_.theta_, 7.0);
        nh.param("map_generator/obstacle/fix_obs_type", generator_params_.fix_obs_type_, 0);
        nh.param("map_generator/obstacle/min_distance", generator_params_.min_dist_, 1.0);

        nh.param("map_generator/pub_rate", generator_params_.pub_rate_, 10.0);
    }

    void MapGenerator::generateWall(std::vector<Eigen::VectorXd> &params,
                                    pcl::PointCloud<pcl::PointXYZ> &cloud_map)
    {
        int x_num_, y_num_, z_num_;
        double x_l_, x_h_, y_l_, y_h_, z_l_, z_h_;
        pcl::PointXYZ pt_;

        for (size_t i = 0; i < params.size(); i++)
        {
            x_l_ = params[i](0);
            x_h_ = params[i](1);
            y_l_ = params[i](2);
            y_h_ = params[i](3);
            z_l_ = params[i](4);
            z_h_ = params[i](5);

            x_num_ = ceil((x_h_ - x_l_) / generator_params_.resolution_);
            y_num_ = ceil((y_h_ - y_l_) / generator_params_.resolution_);
            z_num_ = ceil((z_h_ - z_l_) / generator_params_.resolution_);

            for (int i = 0; i < x_num_; i++)
            {
                for (int j = 0; j < y_num_; j++)
                {
                    for (int k = 0; k < z_num_; k++)
                    {
                        pt_.x = x_l_ + i * generator_params_.resolution_;
                        pt_.y = y_l_ + j * generator_params_.resolution_;
                        pt_.z = z_l_ + k * generator_params_.resolution_;
                        cloud_map.push_back(pt_);
                    }
                }
            }
        }
    }

    void MapGenerator::generateCylinder(std::vector<Eigen::VectorXd> &params,
                                        pcl::PointCloud<pcl::PointXYZ> &cloud_map)
    {
        pcl::PointXYZ pt_;
        double x_, y_, z_, radius_;
        int wid_num_, hei_num_;
        for (size_t i = 0; i < params.size(); i++)
        {
            x_ = floor(params[i](0) / generator_params_.resolution_) * generator_params_.resolution_ + generator_params_.resolution_ / 2.0;
            y_ = floor(params[i](1) / generator_params_.resolution_) * generator_params_.resolution_ + generator_params_.resolution_ / 2.0;
            z_ = floor(params[i](2) / generator_params_.resolution_) * generator_params_.resolution_ + generator_params_.resolution_ / 2.0;
            radius_ = params[i](3) / 2.0;
            hei_num_ = ceil(params[i](4) / generator_params_.resolution_);
            wid_num_ = ceil(params[i](3) / generator_params_.resolution_);

            for (int r = -wid_num_ / 2; r < wid_num_ / 2; r++)
            {
                for (int s = -wid_num_ / 2; s < wid_num_ / 2; s++)
                {
                    if ((r * r + s * s) <= radius_ * radius_)
                    {
                        for (int k = 0; k < hei_num_; k++)
                        {
                            pt_.x = x_ + r * generator_params_.resolution_;
                            pt_.y = y_ + s * generator_params_.resolution_;
                            pt_.z = z_ + k * generator_params_.resolution_;
                            cloud_map.push_back(pt_);
                        }
                    }
                }
            }
        }
    }

    void MapGenerator::generateRing(std::vector<Eigen::VectorXd> &params,
                                    pcl::PointCloud<pcl::PointXYZ> &cloud_map)
    {
        pcl::PointXYZ pt_;
        double x_, y_, z_, radius_x_, radius_y_, theta_;
        Eigen::Vector3d cpt_, translation_;
        Eigen::Matrix3d rotate_;

        for (int i = 0; i < static_cast<int>(params.size()); i++)
        {
            x_ = floor(params[i](0) / generator_params_.resolution_) * generator_params_.resolution_ + generator_params_.resolution_ / 2.0;
            y_ = floor(params[i](1) / generator_params_.resolution_) * generator_params_.resolution_ + generator_params_.resolution_ / 2.0;
            z_ = floor(params[i](2) / generator_params_.resolution_) * generator_params_.resolution_ + generator_params_.resolution_ / 2.0;
            radius_x_ = params[i](3);
            radius_y_ = params[i](4);
            theta_ = params[i](5);

            translation_ = Eigen::Vector3d(x_, y_, z_);
            rotate_ << cos(theta_), -sin(theta_), 0.0, sin(theta_), cos(theta_), 0.0, 0.0, 0.0, 1.0;

            for (double angle_ = 0.0; angle_ < 6.282; angle_ += generator_params_.resolution_ / 2.0)
            {
                cpt_ = rotate_ * Eigen::Vector3d(0.0, radius_x_ * cos(angle_), radius_y_ * sin(angle_)) + translation_;
                pt_.x = cpt_(0);
                pt_.y = cpt_(1);
                pt_.z = cpt_(2);
                cloud_map.push_back(pt_);
            }
        }
    }

    void MapGenerator::generateMap(void)
    {
        std::vector<Eigen::VectorXd> params_;
        int cylinder_num_, wall_num_;
        nh_.param("map_generator/obstacle/cylinder_num", cylinder_num_, 0);
        nh_.param("map_generator/obstacle/wall_num", wall_num_, 0);

        // generate cylinder
        Eigen::VectorXd param_(6);
        for (int i = 0; i < cylinder_num_; i++)
        {
            nh_.param("map_generator/obstacle/cylinder_" + std::to_string(i) + "/x", param_(0), 0.0);
            nh_.param("map_generator/obstacle/cylinder_" + std::to_string(i) + "/y", param_(1), 0.0);
            nh_.param("map_generator/obstacle/cylinder_" + std::to_string(i) + "/z", param_(2), 0.0);
            nh_.param("map_generator/obstacle/cylinder_" + std::to_string(i) + "/radius", param_(3), 0.0);
            nh_.param("map_generator/obstacle/cylinder_" + std::to_string(i) + "/height", param_(4), 0.0);
            params_.push_back(param_);
        }
        generateCylinder(params_, cloud_map_);
        params_.clear();

        // generate wall
        for (int i = 0; i < wall_num_; i++)
        {
            nh_.param("map_generator/obstacle/wall_" + std::to_string(i) + "/x_l", param_(0), 0.0);
            nh_.param("map_generator/obstacle/wall_" + std::to_string(i) + "/x_h", param_(1), 0.0);
            nh_.param("map_generator/obstacle/wall_" + std::to_string(i) + "/y_l", param_(2), 0.0);
            nh_.param("map_generator/obstacle/wall_" + std::to_string(i) + "/y_h", param_(3), 0.0);
            nh_.param("map_generator/obstacle/wall_" + std::to_string(i) + "/z_l", param_(4), 0.0);
            nh_.param("map_generator/obstacle/wall_" + std::to_string(i) + "/z_h", param_(5), 0.0);
            params_.push_back(param_);
        }
        generateWall(params_, cloud_map_);
        params_.clear();

        // std::cout << "checkpoint2.2" << std::endl;
        // // generate ring
        // Eigen::VectorXd param2_(6);
        // param2_ << -8.0, 0.0, 2.5, 0.6, 0.3, 0.0;
        // params_.push_back(param2_);
        // generateRing(params_, cloud_map_);
        // params_.clear();

        // publish map
        cloud_map_.width = cloud_map_.points.size();
        cloud_map_.height = 1;
        cloud_map_.is_dense = true;

        kdtreeLocalMap_.setInputCloud(cloud_map_.makeShared());

        generator_params_.map_ok_ = true;
    }

    void MapGenerator::pubPoints(void)
    {
        while (ros::ok())
        {
            ros::spinOnce();
            if (generator_params_.map_ok_)
                break;
        }
        pcl::toROSMsg(cloud_map_, globalMap_pcd_);
        globalMap_pcd_.header.frame_id = "world";
        map_pub_.publish(globalMap_pcd_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_map_sensing");
    ros::NodeHandle n("~");

    map_generator::MapGenerator::Ptr generator_;

    generator_.reset(new map_generator::MapGenerator());
    generator_->init(n);
    generator_->generateMap();

    ros::Rate loop_rate(1.0);

    // real map
    while (ros::ok())
    {
        generator_->pubPoints();
        ros::spinOnce();
        loop_rate.sleep();
    }
}