#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <string.h>
#include <eigen3/Eigen/Core>
#include <stdio.h>
#include "gridmap.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/kdtree/kdtree_flann.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

using std::placeholders::_1;
using namespace std;

GridMap gmap(300, 300, 0.2, 600, 600);

class OccupancyMapping : public rclcpp::Node
{
    public:
    OccupancyMapping()
    : Node("bof")
    {   tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        tf_broadcaster_->sendTransform(tf_stamped());
        
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ngeeann_av/odom", 20, std::bind(&OccupancyMapping::odom_callback, this, _1));
        subscription2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("velodyne_points", rclcpp::SensorDataQoS(), std::bind(&OccupancyMapping::cloudCallback, this, _1));
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::SensorDataQoS());
    }
   
    private:
    rclcpp::Time now;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double cg2lidar = 2.4;
    // float lat_update_range = 10.0, long_update_range = 30.0;

    std_msgs::msg::Header hd;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription2_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

    geometry_msgs::msg::TransformStamped tf_stamped()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "odom";

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        return t;
    }


    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        hd = msg->header;
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        tf2::Quaternion orientation;
        orientation.setX( msg->pose.pose.orientation.x );
        orientation.setY( msg->pose.pose.orientation.y );
        orientation.setZ( msg->pose.pose.orientation.z );
        orientation.setW( msg->pose.pose.orientation.w );
        theta = tf2::impl::getYaw( orientation );
    }


    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        updateMap(msg);
        nav_msgs::msg::OccupancyGrid occ_map;
        occ_map.header.stamp = hd.stamp;
        occ_map.info.map_load_time = msg->header.stamp;
        gmap.toRosOccMap(occ_map);
        publisher_->publish(occ_map);
    }

    void updateMap( sensor_msgs::msg::PointCloud2::SharedPtr msg )
    {
        const double height = msg->height;
        const double width = msg->width;
        // const double angle_inc = (360 / width);
        // const double range_max = 150;
        // const double range_min = 0.8;


        double R, lidar_x, lidar_y, px, py, pxl, pyl, pyz, angle;


        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*msg, *cloud);

        // for(size_t i = 0; i < cloud->points.size(); i ++){
        for (auto point : cloud->points){

            pxl = point.x;
            pyl = point.y;
            pyz = point.z;

            // std::cout << idx << std::endl;

            // Location of lidar in global frame // lidar offset 
            // lidar_x = x + cg2lidar * -sin(theta);
            // lidar_y = y + cg2lidar * cos(theta);


            if(pyz < 0.3 || pyz > 4) 
            {
                continue;
            }

            // Determines location of point in global frame
            px = pxl*cos(theta) - pyl*sin(theta) + x;
            py = pxl*sin(theta) + pyl*cos(theta) + y;

            gmap.setGridOcc(px, py);

            // if ( (R < range_max) && (r > R) )
            // {
            //     gmap.setGridOcc(px, py);
            //     break;
            // }
            // else
            //     gmap.setGridFree(px, py);
        
        }
    }

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyMapping>());
    rclcpp::shutdown();
    return 0;
}

