#include <ros/ros.h>
#include <vector>
#include <thread>
#include <algorithm>
#include <math.h>
#include <numeric>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
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

#define PI 3.141592

ros::Publisher pub;
ros::Publisher pub_XYZR;
ros::Publisher pub_lidar_flag;
ros::Publisher pub_box;

typedef pcl::PointXYZ PointT;
std::vector <float> distance;

struct Th_Points
{
    float x;
    float y;
    float z;
};
std::vector <Th_Points> Th_Ptr;

namespace LOC{

  struct LIDAR_INPUT
   {
    float x;
    float y;
    float z;
    float r;
   };

  Th_Points Find_Center_Point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obs)
    {
      Th_Points Three_Point_data;
      std::vector<float> X_data; std::vector<float> Y_data; std::vector<float> Z_data;
   
      for(int i=0; i<cloud_obs->points.size();++i)
      { 
        X_data.push_back(cloud_obs->points[i].x);
        Y_data.push_back(cloud_obs->points[i].y);
        Z_data.push_back(cloud_obs->points[i].z);
      }
      for(int k=0; k<X_data.size();k++)
      {
        std::cout << "[" << k <<"] :" <<"("<< X_data[k] <<","<<Y_data[k]<<","<<Z_data[k]<<")" <<std::endl; 
      }
      float X = (std::accumulate(X_data.begin(),X_data.end(),0.0)) /X_data.size();
      float Y = (std::accumulate(Y_data.begin(),Y_data.end(),0.0)) /Y_data.size();
      float Z = (std::accumulate(Z_data.begin(),Z_data.end(),0.0)) /Z_data.size();

      std::cout << "The accumulate point :" <<"("<< X <<","<<Y<<","<<Z<<")" <<std::endl; 

      Three_Point_data = {X,Y,Z};
      X_data.clear(); Y_data.clear();Z_data.clear();
      return Three_Point_data;
    }

  class lidar_obstacle_control{
    private:
         float x;
         float y;
         float z;
         float r;
         int Index;
         float short_distance;
         float angle;
    public:
      void GetObstacleMSG(pcl::PointCloud<pcl::PointXYZI> cloud_obs_I, std::vector<Th_Points> Th_Ptr, std::vector<float> distance)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obs (new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(cloud_obs_I, *cloud_obs);

        Th_Points Three_Center_Points = Find_Center_Point(cloud_obs); 
        float shortest_distance = *std::min_element(distance.begin(),distance.end());
        short_distance = shortest_distance;
        std::vector<float>::iterator sd = std::find(distance.begin(), distance.end(), shortest_distance);

        int index = std::distance(distance.begin(),sd);
        float angle_of_sd = (atan2(Th_Ptr[index].y,Th_Ptr[index].x) * 180) / PI;
        angle = angle_of_sd;
        float Center_distance = sqrt(pow(Three_Center_Points.x,2)+pow(Three_Center_Points.y,2));
        float Radius = std::abs(shortest_distance - Center_distance);
        std::cout << "The shortest point :" <<"("<< Th_Ptr[index].x <<","<<Th_Ptr[index].y<<","<<Th_Ptr[index].z<<")" <<std::endl;

        x=Three_Center_Points.x;
        y=Three_Center_Points.y;
        z=Three_Center_Points.z;
        r=Radius;
      }
      void ShowPointsMSG()
      {
        std::cout << "(x,y,z,r) : " << "("<<x<<","<<y<<","<<z<<","<<r<<")" <<"     ";
        std::cout << " short distance : "<<short_distance<<"     " ;
        std::cout << " angle :" <<angle<<std::endl;
        std::cout << " ----------------------------------------------"<<std::endl;
      }
      LOC::LIDAR_INPUT Give_LIDAR_Input()
      {
        LOC::LIDAR_INPUT Input;
        Input.x =x;
        Input.y=y;
        Input.z=z;
        Input.r=r;
        return Input;
      }

  };
}

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

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
 
  // Data containers used
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_as_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud.makeShared());
  vg.setLeafSize (0.05f, 0.05f, 0.05f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.0, 2.0);
  pass.filter (*cloud_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1.0, 1.0);
  pass.filter (*cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.2, 0.5);
  pass.filter (*cloud_filtered);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(cloud_filtered);
  outrem.setRadiusSearch(0.5);
  outrem.setMinNeighborsInRadius (6);
  outrem.filter (*cloud_filtered);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.08); // 2cm
  ec.setMinClusterSize (20);
  ec.setMaxClusterSize (200);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
 
  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
 
  std_msgs::Bool FLAG;
  pcl::PointCloud<pcl::PointXYZI> TotalCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr TotalCloud_Ptr (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>  TotalCloud_cluster;

  if( cluster_indices.size() != '\0' )
  {
    std::cout << "There is Cluster !!" <<std::endl;
    FLAG.data = true;
    pub_lidar_flag.publish(FLAG);
  }
  else{
    std::cout << "There is No Cluster" <<std::endl;
    FLAG.data = false;
    pub_lidar_flag.publish(FLAG);
  }
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    int k=0;
    for (std::vector<int>::const_iterator pit = it->indices.begin () ;pit != it->indices.end (); ++pit,k++)
    {
        pcl::PointXYZ pt = cloud_filtered->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = (float)(j + 1);
            TotalCloud.push_back(pt2);
            TotalCloud_cluster.push_back(pt2);
            TotalCloud_Ptr->push_back(pt2);

        //std::cout << "(x,y,z) = "<< TotalCloud_Ptr->points[k] << "   This is Ptr   " <<std::endl;
        Th_Points Points = {TotalCloud.points[k].x,TotalCloud.points[k].y,TotalCloud.points[k].z};
        Th_Ptr.push_back(Points);
        //std::cout <<"(x,y,z,) ="<<"("<<Th_Ptr[k].x<<","<<Th_Ptr[k].y<<","<<Th_Ptr[k].z<<")"<<std::endl;
        distance.push_back(sqrt(pow(Th_Ptr[k].x,2)+pow(Th_Ptr[k].y,2))); //+pow(Th_Ptr[k].z,2) 3 dimensional
        //std::cout <<"["<<k<<"]"<< " distance : " << distance[k] <<std::endl;
        //std::cout <<"--------------------------------------"<<std::endl;
    }
    LOC::lidar_obstacle_control Lidar_Controller;
    Lidar_Controller.GetObstacleMSG(TotalCloud,Th_Ptr,distance);
    Lidar_Controller.ShowPointsMSG();
    LOC::LIDAR_INPUT LIDAR_Input = Lidar_Controller.Give_LIDAR_Input();
    geometry_msgs::Vector3 LIDAR_Input_Data;
    LIDAR_Input_Data.x = LIDAR_Input.x;
    LIDAR_Input_Data.y = LIDAR_Input.y;
    LIDAR_Input_Data.z = LIDAR_Input.z;
    pub_XYZR.publish(LIDAR_Input_Data);

    visualization_msgs::Marker Marker = mark_cluster(TotalCloud_Ptr,"marker_name_space",1,0,255,0);
    std::cout << "Marker : " << Marker << std::endl;
    pub_box.publish(Marker);

    //std::cout <<" Angle of shortest distance : " << angle_of_sd <<std::endl;
    //std::cout << (j+1) << " number of cluster, shortest distance : " <<shortest_distance <<std::endl;
    std::cout <<"-------------------"<<(j+1)<< " number of cluster data"<<std::endl;

    Th_Ptr.clear();
    distance.clear();
    TotalCloud.clear();
    TotalCloud_Ptr->clear();
    
    j++;
  }
  
    // Convert To ROS data type 
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud_cluster, cloud_p);
  
  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "velodyne";
  pub.publish(output); 
  ROS_INFO("published it."); 
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "lidar");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  // pub = nh.advertise<pcl_msgs::ModelCoefficients> ("pclplaneoutput", 1);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("clusteroutput", 1);
  pub_XYZR = nh.advertise<geometry_msgs::Vector3> ("LIDAR_XYZR",1);
  pub_lidar_flag = nh.advertise<std_msgs::Bool> ("Object_Detect",1);
  pub_box = nh.advertise<visualization_msgs::Marker> ("Marker",1);

 
  // Spin
  ros::spin ();
}
