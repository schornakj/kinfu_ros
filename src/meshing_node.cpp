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

    ROS_INFO_STREAM("Got TSDF: " << sizeX);
    res.value = true;

    //TODO: Change TSDF to point cloud, do marching cubes, add mesh to service response.

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
