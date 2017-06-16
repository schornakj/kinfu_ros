#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros_rgbd_camera.hpp>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <kinfu_ros/GetMesh.h>
#include <kinfu_ros/GetMeshRequest.h>
#include <kinfu_ros/GetMeshResponse.h>

#include <kinfu_ros/GetTSDF.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
//#include <pcl_visualization/cloud_viewer.h>

using namespace kfusion;

class GenerateMesh
{
public:
  GenerateMesh(ros::NodeHandle& nh)
  {
    tsdf_client_ = nh.serviceClient<kinfu_ros::GetTSDF>("/kinfu/get_tsdf");
    mesh_server_ = nh.advertiseService("get_mesh", &GenerateMesh::GetMesh, this);
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

    pcl::PointCloud<pcl::PointXYZ> cloud;

    GenerateMesh::MakePointCloud(cloud, tsdf_data, size_x, size_y, size_z, num_voxels_x, num_voxels_y, num_voxels_z);

    ROS_INFO("Finished reading point cloud from TSDF");
    // Max cloud size at 512^3 is 134,217,728

    //pcl::io::savePCDFileASCII ("./clouds/test_pcd.pcd", cloud);
    pcl::io::savePCDFileASCII ("/home/jschornak/clouds/test_pcd.pcd", cloud);


    ROS_INFO("Saved point cloud");

//    pcl_visualization::CloudViewer viewer("Simple Cloud Viewer");
//    viewer.showCloud(cloud);
//    while (!viewer.wasStopped())
//    {
//    }

    return true;
  }

  bool MakePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<unsigned int>& data, int size_x, int size_y, int size_z, int res_x, int res_y, int res_z) {
    for (int i = 0; i < res_x; i++) {
      for (int j = 0; j < res_y; j++) {
        for (int k = 0; k < res_z; k++) {
          int currentData = data[res_y*res_z*k + res_y*j + i];
          if (currentData != 0) {
//            ROS_INFO_STREAM("Found an occupied voxel at " << i << " " << j << " " << k);
            pcl::PointXYZ currentPoint(i, j, k);
            cloud.push_back(currentPoint);
          }
        }
      }
    }
    ROS_INFO_STREAM("Made a cloud of size " << cloud.points.size());
    ROS_INFO_STREAM("The first point is at " << cloud.points[0].x << " " << cloud.points[0].y << " " << cloud.points[0].z);
    return true;
  }

private:
  ros::ServiceClient tsdf_client_;
  ros::ServiceServer mesh_server_;


};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "meshing_node");
    //ros::NodeHandle node("~");
    ros::NodeHandle nh;

    GenerateMesh app(nh);
    //app.GetMesh();

    ros::spin();

    return 0;
}
