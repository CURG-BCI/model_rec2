/**
@brief - Wrapper class around surflet object ransac system

This class manages the interaction between the ros service, reading the kinect, and the object recognition system 

*/

#include "ros/ros.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <boost/foreach.hpp>

#include <string>

#include <boost/thread.hpp>


#include <BasicTools/Vtk/VtkTransform.h>
#include <BasicTools/ComputationalGeometry/Algorithms/RANSACPlaneDetector.h>
#include <ObjRecRANSAC/ObjRecRANSAC.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkTransformPolyDataFilter.h>
#include <list>
#include <model_rec2/FindObjects.h>
#include <Eigen/Geometry>
#include <pcl/recognition/obj_rec_ransac.h>

class ModelRec
{

public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef PointCloud::Ptr PointCloudPtr;
  typedef PointCloud::ConstPtr PointCloudConstPtr;
  typedef boost::shared_ptr<UserData> UserDataPtr;
  typedef vtkSmartPointer<vtkPolyDataReader> vtkPolyDataReaderPtr;
  typedef vtkSmartPointer<vtkPolyData> vtkPolyDataPtr;
  
  

  
private:  
  ros::NodeHandle *n_;
  ros::ServiceServer srv_recognizeScene;   
  bool updateCloud();
  bool updatePCLPointCloud();
  bool beginUpdatePCLPointCloud();
  bool updateVTKFromPCLCloud();
  PointCloudPtr pcl_point_cloud_;
  vtkSmartPointer<vtkPoints> vtk_cloud_ptr_;
  vtkSmartPointer<vtkPoints> foreground_vtk_cloud_ptr_;
  vtkSmartPointer<vtkPoints> background_vtk_cloud_ptr_;
  
  std::vector<std::string> model_list_;

  std::string pcl_pointcloud_channel_;
  ros::Subscriber pcl_pointcloud_sub_;
  unsigned int max_cloud_queue_size;
  std::vector<PointCloudConstPtr> cloud_queue;
  void cloudQueuingCallback(const PointCloud::ConstPtr& msg);
  
  bool loadModels();
  /*Plane thickness is in meters here */
  bool removePlane( double plane_thickness = .015, double rel_num_of_plane_points = 0.2)  ;
     
  std::list<UserDataPtr> user_data_list_;
  std::list<vtkPolyDataReaderPtr> readers_;

  pcl::recognition::ObjRecRANSAC objrec_;
  double success_probability_;

  boost::mutex ready_lock_;
  bool vtk_point_cloud_ready_;
  std::list<PointSetShape*> detected_shapes_;
  std::list<vtkPolyDataPtr> scaled_shape_clouds_;
  
  PointCloudPtr loadPointCloudFromPointShape(PointSetShape * shape);
 public:
  /*pair width and voxel size are in meters here */
  ModelRec(ros::NodeHandle* n, std::string pcl_pointcloud_channel, double pair_width = .015, double voxel_size = .004);


  bool runRecognitionCallback(model_rec2::FindObjects::Request & req, model_rec2::FindObjects::Response & res);
};
