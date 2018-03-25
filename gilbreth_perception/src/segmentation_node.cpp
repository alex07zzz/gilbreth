#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <XmlRpcException.h>

// Algorithm params
double down_sample(0.01);
double x_l, x_u, y_l, y_u, z_l, z_u;
bool print_detailed_info(false);
uint32_t g_min_cluster_size = 80;
uint32_t g_max_cluster_size = 25000;
double g_cluster_tolerance = 0.1;

ros::Publisher pub;

bool loadParameter()
{
  try
  {

    // General parameters
    XmlRpc::XmlRpcValue parameter_map;
    XmlRpc::XmlRpcValue camera_roi;
    std::map<std::string, bool> switch_map;
    ros::NodeHandle ph("~");

    ph.getParam("segmentation", parameter_map);
    down_sample = static_cast<double>(parameter_map["down_sample"]);
    g_min_cluster_size = static_cast<int>(parameter_map["min_cluster_size"]);
    g_max_cluster_size = static_cast<int>(parameter_map["max_cluster_size"]);
    g_cluster_tolerance = static_cast<double>(parameter_map["cluster_tolerance"]);

    ph.getParam("segmentation/camera_roi", camera_roi);
    x_l = camera_roi["x"][0];
    x_u = camera_roi["x"][1];
    y_l = camera_roi["y"][0];
    y_u = camera_roi["y"][1];
    z_l = camera_roi["z"][0];
    z_u = camera_roi["z"][1];

    ph.getParam("segmentation/switches", switch_map);
    print_detailed_info = static_cast<bool>(switch_map["print_detailed_info"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("Failed to load segmentation parameters: %s",e.getMessage().c_str());
    return false;
  }
  return true;
}

void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  ros::Time start_time = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_raw(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PassThrough<pcl::PointXYZ> pass;

  pcl::fromROSMsg(*cloud_msg, *scene_raw);
  // Filter the input scene
  pass.setInputCloud(scene_raw);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_l, z_u);
  pass.filter(*scene_filtered);

  pass.setInputCloud(scene_filtered);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_l, x_u);
  pass.filter(*scene_filtered);

  pass.setInputCloud(scene_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(y_l, y_u);
  pass.filter(*scene_filtered);

  if (scene_filtered->points.size() <= 0)
  {
    ROS_WARN("Segmentation found no points after applying passthrough filters");
    return;
  }

  ROS_INFO_COND(print_detailed_info,"Segmentation will process cloud with %lu points",scene_filtered->size());

  // Downsample scene
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(scene_filtered);
  sor.setLeafSize(down_sample, down_sample, down_sample);
  sor.filter(*scene);

  ROS_INFO_COND(print_detailed_info,"Segmentation downsampled scene to %lu points with leafsize %f",scene->size(),down_sample);

  // Cluster Extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(scene);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(g_cluster_tolerance);
  ec.setMinClusterSize(g_min_cluster_size);
  ec.setMaxClusterSize(g_max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(scene);
  ec.extract(cluster_indices);

  if(cluster_indices.empty())
  {
    ROS_ERROR("Segmentation found no clusters after applying Clustering Extraction");
    return;
  }

  // Extracting and publishing cluster
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  for (int i = 0; i < cluster_indices.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    sensor_msgs::PointCloud2 output;

    *indices = cluster_indices[i];
    extract.setInputCloud(scene);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cluster_cloud);

    pcl::toROSMsg(*cluster_cloud, output);
    output.header.stamp = cloud_msg->header.stamp;
    pub.publish(output);

    if (print_detailed_info)
    {
      double elapsed_time = (ros::Time::now() - start_time).toSec();
      ROS_INFO("Segmentation Found %lu clusters in %f seconds",cluster_indices.size(),elapsed_time);
    }
  }

}

int main(int argc, char **argv) {
  // define PCD file subscribed
  // Initialize ROS
  ros::init(argc, argv, "segmentation_node");
  ros::NodeHandle nh;

  if(!loadParameter())
  {
    return -1;
  }

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_1 = nh.subscribe<sensor_msgs::PointCloud2>("scene_point_cloud", 1, cloudCb);
  // ROS publisher
  pub = nh.advertise<sensor_msgs::PointCloud2>("segmentation_result", 10);
  ROS_INFO("Segmentation Node subscribed to %s",pub.getTopic().c_str());
  ROS_INFO("Segmentation Node Ready ...");
  // Spin

  ros::spin();
}
