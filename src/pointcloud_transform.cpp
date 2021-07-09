/**
Usage:
 1. Run node
    rosrun pointcloud_transform pointcloud_transform
 2. Publish transformation as quaternion
    rostopic pub -r 100 /angle geometry_msgs/Quaternion "x: 0.0 y: 0.707 z: 0.0 w: 0.707"
 3. Check new rotated topic
    /laser_cloud_surround_rotated
 */


#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
geometry_msgs::Quaternion quaternion_transformation;
ros::Publisher pointcloud_publisher;


void rotate(const pcl::PointCloud<pcl::PointXYZ>& in_pointcloud, geometry_msgs::Quaternion in_cp){
    //Quaternion input
    const double q0 = in_cp.x;
    const double q1 = in_cp.y;
    const double q2 = in_cp.z;
    const double q3 = in_cp.w;

    //First row of the rotation matrix
    const double r00 = 2 * (q0 * q0 + q1 * q1) - 1;
    const double r01 = 2 * (q1 * q2 - q0 * q3);
    const double r02 = 2 * (q1 * q3 + q0 * q2);

    //Second row of the rotation matrix
    const double r10 = 2 * (q1 * q2 + q0 * q3);
    const double r11 = 2 * (q0 * q0 + q2 * q2) - 1;
    const double r12 = 2 * (q2 * q3 - q0 * q1);

    //Third row of the rotation matrix
    const double r20 = 2 * (q1 * q3 - q0 * q2);
    const double r21 = 2 * (q2 * q3 + q0 * q1);
    const double r22 = 2 * (q0 * q0 + q3 * q3) - 1;

    pcl::PointCloud<pcl::PointXYZ> out_pointcloud;
    Eigen::MatrixXf rotate(4,4);
    rotate << r00, r01, r02, 0,
            r10, r11, r12, 0,
            r20, r21, r22, 0,
            0, 0, 0, 1;
    pcl::transformPointCloud (in_pointcloud, out_pointcloud, rotate);
    pointcloud_publisher.publish(out_pointcloud);
}

void transformation_callback(const geometry_msgs::Quaternion& c_point){
    quaternion_transformation = c_point;
}

void pointcloud_callback(const pcl::PointCloud<pcl::PointXYZ>& pointcloud){
    rotate(pointcloud, quaternion_transformation);
}


int main(int argc,char * argv[])
{
    ros::init(argc, argv, "pointcloud_transform_node");
    ros::NodeHandle nh;
    ros::Subscriber transformation_subscriber = nh.subscribe("/angle", 1, transformation_callback);
    ros::Subscriber pointcloud_subscriber = nh.subscribe("/laser_cloud_surround", 1, pointcloud_callback);
    pointcloud_publisher = nh.advertise<PointCloud>("/laser_cloud_surround_rotated",1);
    ros::spin();
    return 0;
}