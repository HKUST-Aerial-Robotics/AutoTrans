#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

namespace map_generator
{
    class MapGenerator
    {
    public:
        void init(ros::NodeHandle &nh);
        void pubPoints(void);
        void generateMap(void);

        typedef std::unique_ptr<MapGenerator> Ptr;

    private:
        ros::NodeHandle nh_;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap_;

        ros::Publisher map_pub_;

        struct GeneratorParameters
        {
            int seed_;
            int obs_num_, circle_num_;
            double x_size_, y_size_, z_size_;
            double resolution_;

            double x_l_, x_h_, y_l_, y_h_, z_l_, z_h_;
            double radius_l_, radius_h_;
            double w_l_, w_h_, h_l_, h_h_;

            double z_limit_, sensing_range_, pub_rate_;
            double min_dist_;
            int fix_obs_type_;

            bool map_ok_ = false;

            double theta_;
        } generator_params_;

        sensor_msgs::PointCloud2 globalMap_pcd_;
        pcl::PointCloud<pcl::PointXYZ> cloud_map_;

        sensor_msgs::PointCloud2 localMap_pcd_;
        pcl::PointCloud<pcl::PointXYZ> clicked_cloud__;

        void generateWall(std::vector<Eigen::VectorXd> &params,
                          pcl::PointCloud<pcl::PointXYZ> &cloud_map);
        void generateCylinder(std::vector<Eigen::VectorXd> &params,
                              pcl::PointCloud<pcl::PointXYZ> &cloud_map);
        void generateRing(std::vector<Eigen::VectorXd> &params,
                          pcl::PointCloud<pcl::PointXYZ> &cloud_map);
    };
}
