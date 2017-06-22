#include <iostream>
#include <fstream>

#include <bitset>

#include <unistd.h>

//#include <opencv2/imgproc/imgproc.hpp>
//#include <ros/ros_rgbd_camera.hpp>

//#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <kinfu_ros/GetMesh.h>
#include <kinfu_ros/GetMeshRequest.h>
#include <kinfu_ros/GetMeshResponse.h>

#include <kinfu_ros/GetTSDF.h>

#include <ros/half.hpp>
#include <openvdb/openvdb.h>

//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/surface/marching_cubes_hoppe.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/console/parse.h>
//#include <pcl/common/transforms.h>
//#include <pcl/common/projection_matrix.h>



#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/util/Util.h>

//#include <pcl_visualization/cloud_viewer.h>

//using namespace kfusion;

//using half_float::half;

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class GenerateMesh
{
public:
  GenerateMesh(ros::NodeHandle& nh)
  {
    tsdf_client_ = nh.serviceClient<kinfu_ros::GetTSDF>("/kinfu/get_tsdf");
    mesh_server_ = nh.advertiseService("get_mesh", &GenerateMesh::GetMesh, this);
    point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("tsdf_cloud", 1);

    vis_pub_ = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    grid = openvdb::FloatGrid::create(/*background value=*/1.0);

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

    //GenerateMesh::MakePointCloud(cloud, tsdf_data, size_x, size_y, size_z, num_voxels_x, num_voxels_y, num_voxels_z);
    ROS_INFO("Building voxel volume from serialized data...");
    GenerateMesh::MakeVoxelGrid(grid, tsdf_data, size_x, size_y, size_z, num_voxels_x, num_voxels_y, num_voxels_z);
//    ROS_INFO("Making an example sphere...");
//    GenerateMesh::MakeSphereVoxelGrid(grid);
    ROS_INFO("Done building volume");
    //ROS_INFO("Finished reading point cloud from TSDF");
    // Max cloud size at 512^3 is 134,217,728

    //pcl::io::savePCDFileASCII ("./clouds/test_pcd.pcd", cloud);
    //pcl::io::savePCDFileASCII ("/home/jschornak/clouds/test_pcd.pcd", cloud);


   // ROS_INFO("Saved point cloud");

    ROS_INFO("Meshing voxel volume...");
    openvdb::tools::VolumeToMesh mesher;
    mesher.operator()<openvdb::FloatGrid>( grid.operator*() );
    ROS_INFO("Done meshing volume");
    WriteMesh("/home/jschornak/clouds/mesh.obj", mesher);
    //WriteMesh("/home/jschornak/ros/tsdf_ws/src/kinfu_ros/meshes/mesh.obj", mesher);
    ROS_INFO("Saved .obj to file");

//    visualization_msgs::Marker marker;
//    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//    //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
//    marker.mesh_resource = "package://kinfu_ros/meshes/mesh.stl";

//    marker.header.frame_id = "camera_depth_optical_frame";
//    marker.header.stamp = ros::Time();
//    marker.ns = "meshing_node";
//    marker.id = 0;
//   // marker.type = visualization_msgs::Marker::SPHERE;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.pose.position.x = 1;
//    marker.pose.position.y = 1;
//    marker.pose.position.z = 1;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
//    marker.pose.orientation.w = 1.0;
//    marker.scale.x = 1;
//    marker.scale.y = 0.1;
//    marker.scale.z = 0.1;
//    marker.color.a = 1.0; // Don't forget to set the alpha!
//    marker.color.r = 0.0;
//    marker.color.g = 1.0;
//    marker.color.b = 0.0;

//    vis_pub_.publish(marker);

    //ROS_INFO("Updated point cloud publisher");
    //sensor_msgs::PointCloud2 cloud_msg;
    //pcl::toROSMsg(cloud, cloud_msg);
    //std::string frame_id = "camera_depth_optical_frame";
    //cloud_msg.header.frame_id = cloud_msg.header.frame_id.empty() ? frame_id : cloud_msg.header.frame_id;
    //point_cloud_publisher_.publish(cloud_msg);

    return true;
  }



  bool GetTSDFData(unsigned int input,  half_float::half& voxelValue, uint16_t& voxelWeight) {
    std::bitset<32> inputBits(input);

//    std::bitset<16> valueBits;;
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

//    std::bitset<16> weightBits;
//    weightBits[0] = inputBits[16];
//    weightBits[1] = inputBits[17];
//    weightBits[2] = inputBits[18];
//    weightBits[3] = inputBits[19];
//    weightBits[4] = inputBits[20];
//    weightBits[5] = inputBits[21];
//    weightBits[6] = inputBits[22];
//    weightBits[7] = inputBits[23];
//    weightBits[8] = inputBits[24];
//    weightBits[9] = inputBits[25];
//    weightBits[10] = inputBits[26];
//    weightBits[11] = inputBits[27];
//    weightBits[12] = inputBits[28];
//    weightBits[13] = inputBits[29];
//    weightBits[14] = inputBits[30];
//    weightBits[15] = inputBits[31];

    //voxelValue = reinterpret_cast<half>(valueBits.to_ulong());
//    voxelWeight = (int)weightBits.to_ulong();

    std::memcpy(&voxelValue, &input, 2);
    std::memcpy(&voxelWeight, &input + 1, 2);

//    ROS_INFO_STREAM("Voxel bits: " << inputBits);
//    ROS_INFO_STREAM("Voxel ints: value:" << voxelValue << " weight:" <<voxelWeight);
    return true;
  }

//  bool MakePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<unsigned int>& data, int size_x, int size_y, int size_z, int res_x, int res_y, int res_z) {
//    float voxel_dim_x = (float)size_x/(float)res_x;
//    float voxel_dim_y = (float)size_y/(float)res_y;
//    float voxel_dim_z = (float)size_z/(float)res_z;

//    ROS_INFO_STREAM("Voxel X dim: " << voxel_dim_x);

//    for (int i = 0; i < res_x; i++) {
//      for (int j = 0; j < res_y; j++) {
//        for (int k = 0; k < res_z; k++) {
//          int currentData = data[res_y*res_z*k + res_y*j + i];
//          half_float::half currentValue;
//          int currentWeight;

//          GetTSDFData(currentData, currentValue, currentWeight);
//          if (currentWeight > 40 && currentValue < 0.1 && currentValue > -0.1) {
//            //ROS_INFO_STREAM("Found an occupied voxel at (" << i << ", " << j << ", " << k << ")");

//            pcl::PointXYZ currentPoint((float)i*voxel_dim_x, (float)j*voxel_dim_y, (float)k*voxel_dim_z);
//            cloud.push_back(currentPoint);
//            //usleep(100000);
//          }
//        }
//      }
//    }
//    ROS_INFO_STREAM("Made a cloud of size " << cloud.points.size());
//    ROS_INFO_STREAM("The first point is at " << cloud.points[0].x << " " << cloud.points[0].y << " " << cloud.points[0].z);

//    return true;
//  }

  bool MakeVoxelGrid(openvdb::FloatGrid::Ptr& grid, std::vector<unsigned int>& data, int size_x, int size_y, int size_z, int res_x, int res_y, int res_z) {
    typedef typename openvdb::FloatGrid::ValueType ValueT;

    openvdb::FloatGrid::Accessor accessor = grid->getAccessor();
    //openvdb::FloatGrid::Ptr grid = openvdb::FloatGrid::create(/*background value=*/2.0);

    float voxel_dim_x = (float)size_x/(float)res_x;
    //float voxel_dim_y = (float)size_y/(float)res_y;
    //float voxel_dim_z = (float)size_z/(float)res_z;

    ROS_INFO_STREAM("Meters per voxel: " << voxel_dim_x);

    openvdb::Coord ijk;
    int &i = ijk[0], &j = ijk[1], &k = ijk[2];

    for (i = 0; i < res_x; i++) {
      for (j = 0; j < res_y; j++) {
        for (k = 0; k < res_z; k++) {
          int currentData = data[res_y*res_z*k + res_y*j + i];
          half_float::half currentValue;
          //float currentValue;

          uint16_t currentWeight;

          GetTSDFData(currentData, currentValue, currentWeight);
          ValueT val = ValueT(currentValue);
//          ROS_INFO_STREAM("Current weight: " << currentWeight);
          accessor.setValue(ijk, val);
          }
        }
      }
    grid->setTransform(openvdb::math::Transform::createLinearTransform(/*voxel size=*/voxel_dim_x));
    }




    bool MakeSphereVoxelGrid(openvdb::FloatGrid::Ptr& grid) {
      grid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(1.0, openvdb::Vec3f(0,0,0), /*voxel size=*/0.5, /*width=*/4.0);
    }

    //pcl::PointCloud<pcl::PointXYZ> cloud;
    openvdb::FloatGrid::Ptr grid;



private:
  void WriteMesh(const char* filename,
      openvdb::tools::VolumeToMesh &mesh ){

    std::ofstream file;
    file.open(filename);

    openvdb::tools::PointList *verts = &mesh.pointList();
    openvdb::tools::PolygonPoolList *polys = &mesh.polygonPoolList();

    for( size_t i = 0; i < mesh.pointListSize(); i++ ){
      openvdb::Vec3s &v = (*verts)[i];
      file << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
    }

    for( size_t i = 0; i < mesh.polygonPoolListSize(); i++ ){

      for( size_t ndx = 0; ndx < (*polys)[i].numTriangles(); ndx++ ){
        openvdb::Vec3I *p = &((*polys)[i].triangle(ndx));
        file << "f " << p->x()+1 << " " << p->y()+1 << " " << p->z()+1 << std::endl;
      }

      for( size_t ndx = 0; ndx < (*polys)[i].numQuads(); ndx++ ){
        openvdb::Vec4I *p = &((*polys)[i].quad(ndx));
        file << "f " << p->x()+1 << " " << p->y()+1 << " " << p->z()+1 << " " << p->w()+1 << std::endl;
      }
    }

    file.close();
  }


  ros::ServiceClient tsdf_client_;
  ros::ServiceServer mesh_server_;
  ros::Publisher point_cloud_publisher_;
  ros::Publisher vis_pub_;

};

/*
// Populate the given grid with a narrow-band level set representation of a sphere.
// The width of the narrow band is determined by the grid's background value.
// (Example code only; use tools::createSphereSDF() in production.)
template<class GridType>
void setTSDFGridValue(GridType& grid, const float& distance, int voxelCoordX, int voxelCoordY, int voxelCoordZ)
{
    typedef typename GridType::ValueType ValueT;
    // Distance value for the constant region exterior to the narrow band
    //const ValueT outside = grid.background();
    // Distance value for the constant region interior to the narrow band
    // (by convention, the signed distance is negative in the interior of
    // a level set)
    //const ValueT inside = -outside;
    // Use the background value as the width in voxels of the narrow band.
    // (The narrow band is centered on the surface of the sphere, which
    // has distance 0.)
    //int padding = int(openvdb::math::RoundUp(openvdb::math::Abs(outside)));
    // The bounding box of the narrow band is 2*dim voxels on a side.
    //int dim = int(radius + padding);
    // Get a voxel accessor.
    typename GridType::Accessor accessor = grid.getAccessor();
    // Compute the signed distance from the surface of the sphere of each
    // voxel within the bounding box and insert the value into the grid
    // if it is smaller in magnitude than the background value.
    openvdb::Coord ijk;
    int &i = ijk[0], &j = ijk[1], &k = ijk[2];

    i = voxelCoordX;
    j = voxelCoordY;
    k = voxelCoordZ;
    ValueT val = ValueT(distance);
    accessor.setValue(ijk, val);


//    for (i = 0; i < voxels_x; ++i) {
//        const float x2 = openvdb::math::Pow2(i - c[0]);
//        for (j = 0; j < voxels_y; ++j) {
//            //const float x2y2 = openvdb::math::Pow2(j - c[1]) + x2;
//            for (k = 0; k < voxels_z; ++k) {
//                // The distance from the sphere surface in voxels
//                const float dist = openvdb::math::Sqrt(x2y2 + openvdb::math::Pow2(k - c[2])) - radius;
//                // Convert the floating-point distance to the grid's value type.
//                ValueT val = ValueT(dist);
//                // Only insert distances that are smaller in magnitude than
//                // the background value.
//                //if (val < inside || outside < val) continue;
//                // Set the distance for voxel (i,j,k).
//                accessor.setValue(ijk, val);
//            }
//        }
//    }

    // Propagate the outside/inside sign information from the narrow band
    // throughout the grid.
    //openvdb::tools::signedFloodFill(grid.tree());
}
*/

int main(int argc, char* argv[])
{
    openvdb::initialize();
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
