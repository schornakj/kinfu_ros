#include <iostream>
#include <bitset>

#include <unistd.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros_rgbd_camera.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <kinfu_ros/GetMesh.h>
#include <kinfu_ros/GetMeshRequest.h>
#include <kinfu_ros/GetMeshResponse.h>

#include <kinfu_ros/GetTSDF.h>

#include <ros/half.hpp>

#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/projection_matrix.h>
//#include <pcl_visualization/cloud_viewer.h>

using namespace kfusion;

using half_float::half;

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class GenerateMesh
{
public:
  GenerateMesh(ros::NodeHandle& nh)
  {
    tsdf_client_ = nh.serviceClient<kinfu_ros::GetTSDF>("/kinfu/get_tsdf");
    mesh_server_ = nh.advertiseService("get_mesh", &GenerateMesh::GetMesh, this);
    point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("tsdf_cloud", 1);

  }

  bool GetMesh(kinfu_ros::GetMeshRequest& req, kinfu_ros::GetMeshResponse& res) {
    ROS_INFO("Attempting to get TSDF");
    kinfu_ros::GetTSDF srv;
    if (!tsdf_client_.call(srv))
    {
      ROS_ERROR("Couldn't get TSDF");
      return false;
    }

    int sizeX = srv.response.tsdf.size_x;

    ROS_INFO("Got TSDF");
    res.value = true;

    //TODO: Change TSDF to point cloud, do marching cubes, add mesh to service response.

    int size_x = srv.response.tsdf.size_x;
    int size_y = srv.response.tsdf.size_y;
    int size_z = srv.response.tsdf.size_z;

    int num_voxels_x = srv.response.tsdf.num_voxels_x;
    int num_voxels_y = srv.response.tsdf.num_voxels_y;
    int num_voxels_z = srv.response.tsdf.num_voxels_z;

    int length = num_voxels_x*num_voxels_y*num_voxels_y;

    std::vector<unsigned int> tsdf_data = srv.response.tsdf.data;

    ROS_INFO("Got volume info from TSDF");
    ROS_INFO_STREAM("Max weight: " << srv.response.tsdf.max_weight);



    GenerateMesh::MakePointCloud(cloud, tsdf_data, size_x, size_y, size_z, num_voxels_x, num_voxels_y, num_voxels_z);

    ROS_INFO("Finished reading point cloud from TSDF");
    // Max cloud size at 512^3 is 134,217,728

    //pcl::io::savePCDFileASCII ("./clouds/test_pcd.pcd", cloud);
    pcl::io::savePCDFileASCII ("/home/jschornak/clouds/test_pcd.pcd", cloud);


    ROS_INFO("Saved point cloud");


    ROS_INFO("Updated point cloud publisher");
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    std::string frame_id = "camera_depth_optical_frame";
    cloud_msg.header.frame_id = cloud_msg.header.frame_id.empty() ? frame_id : cloud_msg.header.frame_id;
    point_cloud_publisher_.publish(cloud_msg);

//    pcl_visualization::CloudViewer viewer("Simple Cloud Viewer");
//    viewer.showCloud(cloud);
//    while (!viewer.wasStopped())
//    {
//    }

    return true;
  }



  bool GetTSDFData(unsigned int input, half& voxelValue, int& voxelWeight) {
    std::bitset<32> inputBits(input);


//    std::bitset<16> valueBits;
//    valueBits[0] = inputBits[0];
//    valueBits[1] = inputBits[1];
//    valueBits[2] = inputBits[2];
//    valueBits[3] = inputBits[3];
//    valueBits[4] = inputBits[4];
//    valueBits[5] = inputBits[5];
//    valueBits[6] = inputBits[6];
//    valueBits[7] = inputBits[7];
//    valueBits[8] = inputBits[8];
//    valueBits[9] = inputBits[9];
//    valueBits[10] = inputBits[10];
//    valueBits[11] = inputBits[11];
//    valueBits[12] = inputBits[12];
//    valueBits[13] = inputBits[13];
//    valueBits[14] = inputBits[14];
//    valueBits[15] = inputBits[15];

    std::bitset<16> weightBits;
    weightBits[0] = inputBits[16];
    weightBits[1] = inputBits[17];
    weightBits[2] = inputBits[18];
    weightBits[3] = inputBits[19];
    weightBits[4] = inputBits[20];
    weightBits[5] = inputBits[21];
    weightBits[6] = inputBits[22];
    weightBits[7] = inputBits[23];
    weightBits[8] = inputBits[24];
    weightBits[9] = inputBits[25];
    weightBits[10] = inputBits[26];
    weightBits[11] = inputBits[27];
    weightBits[12] = inputBits[28];
    weightBits[13] = inputBits[29];
    weightBits[14] = inputBits[30];
    weightBits[15] = inputBits[31];

    //voxelValue = reinterpret_cast<half>(valueBits.to_ulong());
    voxelWeight = (int)weightBits.to_ulong();

    std::memcpy(&voxelValue, &input, 2);
//    std::memcpy(&voxelWeight, &input + 4, 2);

    //ROS_INFO_STREAM("Voxel bits: value:" << valueBits << " weight:" <<weightBits);
    //ROS_INFO_STREAM("Voxel ints: value:" << voxelValue << " weight:" <<voxelWeight);
    return true;
  }

  bool MakePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<unsigned int>& data, int size_x, int size_y, int size_z, int res_x, int res_y, int res_z) {
    float voxel_dim_x = (float)size_x/(float)res_x;
    float voxel_dim_y = (float)size_y/(float)res_y;
    float voxel_dim_z = (float)size_z/(float)res_z;

    ROS_INFO_STREAM("Voxel X dim: " << voxel_dim_x);

    for (int i = 0; i < res_x; i++) {
      for (int j = 0; j < res_y; j++) {
        for (int k = 0; k < res_z; k++) {
          int currentData = data[res_y*res_z*k + res_y*j + i];
          half currentValue;
          int currentWeight;

          GetTSDFData(currentData, currentValue, currentWeight);
          if (currentWeight > 40 && currentValue < 0.1 && currentValue > -0.1) {
            //ROS_INFO_STREAM("Found an occupied voxel at (" << i << ", " << j << ", " << k << ")");

            pcl::PointXYZ currentPoint((float)i*voxel_dim_x, (float)j*voxel_dim_y, (float)k*voxel_dim_z);
            cloud.push_back(currentPoint);
            //usleep(100000);
          }
        }
      }
    }
    ROS_INFO_STREAM("Made a cloud of size " << cloud.points.size());
    ROS_INFO_STREAM("The first point is at " << cloud.points[0].x << " " << cloud.points[0].y << " " << cloud.points[0].z);

    return true;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;

private:
  ros::ServiceClient tsdf_client_;
  ros::ServiceServer mesh_server_;
  ros::Publisher point_cloud_publisher_;

};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "meshing_node");
    //ros::NodeHandle node("~");
    ros::NodeHandle nh;

    GenerateMesh app(nh);
    //app.GetMesh();

    ros::spin();
//    ros::AsyncSpinner spinner(2);
//    spinner.start();

//    ros::Publisher point_cloud_publisher_;
//    point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("tsdf_cloud", 1);

//    ROS_INFO("Publishing point cloud...");

//    //ROS_INFO_STREAM("Cloud data " << cloud_msg.data);
//    while(ros::ok()) {
//      sensor_msgs::PointCloud2 cloud_msg;
//      pcl::toROSMsg(app.cloud, cloud_msg);

//      std::string frame_id = "camera_depth_optical_frame";
//      cloud_msg.header.frame_id = cloud_msg.header.frame_id.empty() ? frame_id : cloud_msg.header.frame_id;
//      point_cloud_publisher_.publish(cloud_msg);
//    }

//    ros::waitForShutdown();

    return 0;
}
