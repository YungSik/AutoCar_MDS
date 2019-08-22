#include <ros/ros.h>
#include <vector>
#include <thread>
#include <algorithm>
#include <math.h>
#include <numeric>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

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

class MB
{
private:
    std_msgs::Bool LIDAR_FLAG;
    geometry_msgs::Vector3 XYZR_POINTS;
public:
    void MB_callback_lidar_flag(const std_msgs::Bool& Lidar_Flag );
    void MB_callback_lidar_xyz(const geometry_msgs::Vector3::ConstPtr& XYZR_Points );
    void ShowMB();
};
void MB::MB_callback_lidar_flag(const std_msgs::Bool& Lidar_Flag )
{
    std::cout << "Is There a Cluster ? -----> " ;
        if (Lidar_Flag.data)
        {
            //std::cout << " Cluster Exist!" << Lidar_Flag.data << std::endl;
            LIDAR_FLAG.data = Lidar_Flag.data;
            std::cout << " Cluster Exist!" << LIDAR_FLAG.data << std::endl;
        }
        else
        {
            //std::cout << " Cluster Not Exist!" << Lidar_Flag.data << std::endl;
            LIDAR_FLAG.data = Lidar_Flag.data;
            std::cout << " Cluster Not Exist!" << LIDAR_FLAG.data << std::endl;
        }  
}
void MB::MB_callback_lidar_xyz(const geometry_msgs::Vector3::ConstPtr& XYZR_Points )
{
        //std::cout << "Subscribe success!!" << std::endl;
        //std::cout << XYZR_Points->x << XYZR_Points->y << XYZR_Points->z <<std::endl;
        XYZR_POINTS.x = XYZR_Points->x;
        XYZR_POINTS.y = XYZR_Points->y;
        XYZR_POINTS.z = XYZR_Points->z; 
        std::cout << "XYZR POINTS : " << "("<<XYZR_POINTS.x<<"," << XYZR_POINTS.y <<","<< XYZR_POINTS.z<<")" <<std::endl;
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
//     {
//         std::cout << " Cluster Exist!" << Lidar_Flag.data << std::endl;
//     }
//     else
//     {
//         std::cout << " Cluster Not Exist!" << Lidar_Flag.data << std::endl;
//     }
    
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
 
    ros::Subscriber sub_flag = n.subscribe("Object_Detect",1,&MB::MB_callback_lidar_flag,&mb);
    ros::Subscriber sub = n.subscribe("LIDAR_XYZR",1,&MB::MB_callback_lidar_xyz,&mb);
       
    mb.ShowMB();
   
    ros::spin();

    return 0;
}