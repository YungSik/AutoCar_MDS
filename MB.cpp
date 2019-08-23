#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <thread>
#include <algorithm>
#include <math.h>
#include <numeric>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <lidar/ControlCommand.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>

visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b)
{
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;

  pcl::compute3DCentroid (*cloud_cluster, centroid);
  pcl::getMinMax3D (*cloud_cluster, min, max);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "velodyne";
  marker.header.stamp = ros::Time::now();

  marker.ns = ns;
  marker.id = id;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = centroid[0];
  marker.pose.position.y = centroid[1];
  marker.pose.position.z = centroid[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = (max[0]-min[0]);
  marker.scale.y = (max[1]-min[1]);
  marker.scale.z = (max[2]-min[2]);

  if (marker.scale.x ==0)
      marker.scale.x=0.1;

  if (marker.scale.y ==0)
    marker.scale.y=0.1;

  if (marker.scale.z ==0)
    marker.scale.z=0.1;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5;

  //marker.lifetime = ros::Duration();
  marker.lifetime = ros::Duration(300);
  return marker;
}

ros::Publisher pub_Driving;

class MB
{
private:
    std_msgs::Bool LIDAR_FLAG;
    geometry_msgs::Vector3 XYZR_POINTS;
    std_msgs::Float32 Steer_C;
public:
    void MB_callback_lidar_flag(const std_msgs::Bool& Lidar_Flag );
    void MB_callback_lidar_xyz(const pcl::PointCloud<pcl::PointXYZI>& pointcloud );
    void MB_callback_steer_camera(const std_msgs::Float32& steer_c);
    void ShowMB();
};

void MB::MB_callback_lidar_flag(const std_msgs::Bool& Lidar_Flag )
{
    std::cout << "Is There a Cluster ? -----> " ;
        if (Lidar_Flag.data)
        {
            //std::cout << " Cluster Exist!" << Lidar_Flag.data << std::endl;
            LIDAR_FLAG = Lidar_Flag;
            std::cout << " Cluster Exist!" << LIDAR_FLAG.data << std::endl;
        }
        else
        {
            //std::cout << " Cluster Not Exist!" << Lidar_Flag.data << std::endl;
            LIDAR_FLAG = Lidar_Flag;
            std::cout << " Cluster Not Exist!" << LIDAR_FLAG.data << std::endl;
        }
}
void MB::MB_callback_lidar_xyz(const pcl::PointCloud<pcl::PointXYZI>& pointcloud)
{
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pointcloud_ptr->resize(pointcloud.size());

         for(int i=0;i < pointcloud.points.size(); ++i)
         {
             pointcloud_ptr->points[i].x = pointcloud.points[i].x;
             pointcloud_ptr->points[i].y = pointcloud.points[i].y;
             pointcloud_ptr->points[i].z = pointcloud.points[i].z;
         }
         visualization_msgs::Marker Marker = mark_cluster(pointcloud_ptr,"marker_name_space",1,0,255,0);
        //std::cout << "Marker : " << Marker << std::endl;

          XYZR_POINTS.x = Marker.pose.position.x ;
          XYZR_POINTS.y = Marker.pose.position.y ;
          XYZR_POINTS.z = Marker.pose.position.z ;
          std::cout << "XYZR POINTS : " << "("<<XYZR_POINTS.x<<"," << XYZR_POINTS.y <<","<< XYZR_POINTS.z<<")" <<std::endl;
}
void MB::MB_callback_steer_camera(const std_msgs::Float32& steer_c)
{
  Steer_C.data = steer_c.data;
  std::cout << "Steer_Value :" << Steer_C <<std::endl;
}
void MB::ShowMB()
{
    std::cout << "LIDAR BOOLEAN : " << LIDAR_FLAG.data ;
    std::cout << "XYZR POINTS : " << "("<<XYZR_POINTS.x<<"," << XYZR_POINTS.y <<","<< XYZR_POINTS.z<<")" <<std::endl;
}

// void MB_callback_lidar_flag(const std_msgs::Bool& Lidar_Flag )
// {
//     std::cout << "Is There a Cluster ? -----> " ;
//     if (Lidar_Flag.data)
//     { std::cout << " Cluster Exist!" << Lidar_Flag.data << std::endl; }
//     else
//     { std::cout << " Cluster Not Exist!" << Lidar_Flag.data << std::endl; }
// }
// void MB_callback_lidar_xyz(const geometry_msgs::Vector3::ConstPtr& XYZR_Points )
// {
//     std::cout << "Subscribe success!!" << std::endl;
//     std::cout << XYZR_Points->x << XYZR_Points->y << XYZR_Points->z <<std::endl;
// }

int main(int argc, char **argv)
{

    ros::init(argc,argv,"MB");
    ros::NodeHandle n;
    MB mb;
    lidar::ControlCommand output;
    pub_Driving = n.advertise<lidar::ControlCommand> ("DRIVING_MODE",1);

    ros::Subscriber sub_flag = n.subscribe("Object_Detect",1, &MB::MB_callback_lidar_flag, &mb);
    ros::Subscriber sub = n.subscribe("LIDAR_POINTCLOUD",100, &MB::MB_callback_lidar_xyz, &mb);
    ros::Subscriber sub_camera = n.subscribe("STEER_C",1,&MB::MB_callback_steer_camera, &mb);

    while (ros ::ok())
    {
        output.Gear =1; output.Speed =1; output.Steer_G =1; output.Steer_C =1; output.Brake =1; 
        pub_Driving.publish(output);
        //mb.ShowMB();
        ros::spinOnce();
    }

    return 0;
}
