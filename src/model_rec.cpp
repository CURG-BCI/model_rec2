#include "model_rec.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl_ros/transforms.h>
#include "/opt/ros/electric/stacks/perception_pcl/pcl/include/pcl-1.1/pcl/registration/transforms.h"
#include <pcl_ros/filters/filter.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/PointCloud2.h>


ModelRec::ModelRec(ros::NodeHandle* n, std::string pcl_pointcloud_channel, double pair_width, double voxel_size): n_(n), foreground_vtk_cloud_ptr_(vtkSmartPointer<vtkPoints>::New()), background_vtk_cloud_ptr_(vtkSmartPointer<vtkPoints>::New()), pcl_pointcloud_channel_(pcl_pointcloud_channel), max_cloud_queue_size(2), objrec_(pair_width, voxel_size, 0.5), success_probability_(0.99) 
{

  srv_recognizeScene = n_->advertiseService("recognize_objects", &ModelRec::runRecognitionCallback, this);
  model_list_.push_back("all");
  model_list_.push_back("garnier_shampoo_bottle");
  model_list_.push_back("gillette_shaving_gel");


  loadModels();
  objrec_.setVisibility(0.1);
  objrec_.setRelativeObjectSize(0.1);
  objrec_.setNumberOfThreads(4);  
}


/*@brief - Update internal pcl pointcloud by listening to point cloud stream
  
 */
bool
ModelRec::beginUpdatePCLPointCloud()
{

  ROS_INFO("Entering begin update\n");
  cloud_queue.clear();
  pcl_point_cloud_.reset(new PointCloud);
  pcl_pointcloud_sub_ = n_->subscribe<PointCloud>(pcl_pointcloud_channel_, 1, &ModelRec::cloudQueuingCallback, this);
  return true;
}


void
ModelRec::cloudQueuingCallback(const PointCloud::ConstPtr& msg)
{
  PointCloudPtr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  ROS_INFO("queuing cloud\n");
  std::cout <<" cloud message stats" << msg << std::endl;
  //probably take a mutex here 
  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (msg);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);
  */
  std::vector<int> indeces;  
  pcl::removeNaNFromPointCloud (*msg, *cloud_filtered, indeces);
  cloud_queue.push_back(cloud_filtered);
  std::cout << "Cloud " <<cloud_queue.size() << " : " << (*cloud_filtered)[1].x << " "  <<(*cloud_filtered)[1].y << " " <<(*cloud_filtered)[1].z << std::endl;
  if (cloud_queue.size() == max_cloud_queue_size)
    {
      pcl_pointcloud_sub_.shutdown();
      updateCloud();
    }
  
}

bool
ModelRec::updatePCLPointCloud()
{
  ROS_INFO("concatenating cloud\n");
  pcl_point_cloud_->header.frame_id=cloud_queue[0]->header.frame_id;
  int cloud_count = 0;
  BOOST_FOREACH(PointCloudConstPtr ptr , cloud_queue)
    {
      ROS_INFO("adding cloud %i\n",cloud_count);
      cloud_count ++;
      (*pcl_point_cloud_) += (*ptr);
    }
  ROS_INFO("finished concatenating cloud\n");
  return true;
}

bool ModelRec::updateVTKFromPCLCloud(){
  ROS_INFO("transforming cloud to vtk\n");
  boost::lock_guard<boost::mutex> lock(ready_lock_);  
  PointCloudPtr scaled_cloud_ptr(new PointCloud);
  Eigen::Matrix4f transform;
  transform.setIdentity();
  transform(0,0) = 1000.0;
  transform(1,1) = 1000.0;
  transform(2,2) = 1000.0;
    std::cout << "Cloud 1: " << (*pcl_point_cloud_)[1].x << " "  <<(*pcl_point_cloud_)[1].y << " " <<(*pcl_point_cloud_)[1].z << std::endl;

  pcl::transformPointCloud<pcl::PointXYZ>(*pcl_point_cloud_,  *scaled_cloud_ptr, transform);

  std::cout << "Cloud 2: " << (*scaled_cloud_ptr)[1].x << " "  <<(*scaled_cloud_ptr)[1].y << " " <<(*scaled_cloud_ptr)[1].z << std::endl;
  pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PointXYZ> pgh(scaled_cloud_ptr);
  pgh.getGeometry(vtk_cloud_ptr_); 
  vtk_point_cloud_ready_ = true;
  return true;
}


bool 
ModelRec::updateCloud()
{
    ROS_INFO("updating all clouds\n");
    updatePCLPointCloud();
     ROS_INFO("returned from update pcl cloud\n");
    updateVTKFromPCLCloud(); 
    ROS_INFO("returned from update vtk cloud\n");
    return true;
}


bool
ModelRec::loadModels()
{
	char fileName[1024];
	// Derive the class 'UserData' if you want to save some specific information
	// about each model. When you load a model in the library you have to pass a 'UserData'-pointer
	// to the method 'addModel()'. If the corresponding model is detected in the scene, you can use
	// the 'UserData'-pointer which is returned by the recognition method, in order to know which
	// model has been detected.

	BOOST_FOREACH(std::string s, model_list_)
	  {

	    // Create a user object
	    UserDataPtr userData(new UserData());
	    userData->setLabel(s.c_str());
	    // Load the model
	    sprintf(fileName, ("/home/armuser/ros/model_rec/Recognition/data/" + s + ".vtk").c_str());
	    vtkPolyDataReaderPtr reader(vtkPolyDataReader::New());
	    reader->SetFileName(fileName);
	    reader->Update();
	    // Add the model to the model library
	    objrec_.addModel(reader->GetOutput(), userData.get());
	    // Save the user data and the reader in order to delete them later (outside this function)
	    user_data_list_.push_back(userData);
	    readers_.push_back(reader);
	  }
	return true;
}


bool 
ModelRec::removePlane( double plane_thickness, double rel_num_of_plane_points)  
{
	RANSACPlaneDetector plane_detector;
	// Perform the plane detection
	plane_detector.detectPlane(vtk_cloud_ptr_, rel_num_of_plane_points, plane_thickness);
	// Check the orientation of the detected plane normal
	if ( plane_detector.getPlaneNormal()[2] > 0.0 )
		plane_detector.flipPlaneNormal();
	// Get the points above the plane (the scene) and the ones below it (background)
	plane_detector.getPointsAbovePlane(foreground_vtk_cloud_ptr_, background_vtk_cloud_ptr_);
	ROS_INFO("Number of foreground points: %i\n", foreground_vtk_cloud_ptr_->GetNumberOfPoints());
	ROS_INFO("Number of background points: %i\n", background_vtk_cloud_ptr_->GetNumberOfPoints());
	return true;
}

void
eigenFromCArray(const double *in, Eigen::Affine3d &out){
  out(0,0) = in[0]; 
  out(0,1) = in[1];
  out(0,2) = in[2];
  out(1,0) = in[3];
  out(1,1) = in[4];
  out(1,2) = in[5];
  out(2,0) = in[6];
  out(2,1) = in[7];
  out(2,2) = in[8];
  out(0,3) = in[9];
  out(1,3) = in[10];
  out(2,3) = in[11];
}


bool
ModelRec::runRecognitionCallback(model_rec::FindObjects::Request & req, model_rec::FindObjects::Response & res)
{
  ROS_INFO("Entering callback\n");
  ready_lock_.lock();
  vtk_point_cloud_ready_ = false;
  ready_lock_.unlock();
  beginUpdatePCLPointCloud();
  ros::Rate r(10);
  while(!vtk_point_cloud_ready_)
    {//spin
      r.sleep();
      ros::spinOnce();
    }
  ROS_INFO("Finished acquiring images\n");
  if(1){
    removePlane();
    objrec_.doRecognition(foreground_vtk_cloud_ptr_, success_probability_, detected_shapes_);
  }
  else{
    ROS_INFO("Number of points: %i\n", vtk_cloud_ptr_->GetNumberOfPoints());
    objrec_.doRecognition(vtk_cloud_ptr_, success_probability_, detected_shapes_);
  }
  ROS_INFO("Number of shapes: %i\n", detected_shapes_.size());
  BOOST_FOREACH(PointSetShape * shape, detected_shapes_){
    geometry_msgs::Pose shape_pose_msg;
    Eigen::Affine3d shape_pose;
    shape_pose.setIdentity();
    const double *mat4x4 = shape->getRigidTransform();
    eigenFromCArray(mat4x4, shape_pose);
    tf::poseEigenToMsg(shape_pose, shape_pose_msg);
    shape_pose_msg.position.x/=1000.0; shape_pose_msg.position.y/=1000.0; shape_pose_msg.position.z/=1000.0;
    std::cout << "Shape name " << shape->getUserData()->getLabel() << "\n"
	      << "Transform " << shape_pose_msg << "\n";
    res.object_name.push_back(std::string(shape->getUserData()->getLabel()));
    res.object_pose.push_back(shape_pose_msg);
    PointCloudPtr pc = loadPointCloudFromPointShape(shape);
    pc->header.frame_id = shape->getUserData()->getLabel();
    sensor_msgs::PointCloud2 p;
    pcl::toROSMsg(*pc, p);    
    res.pointcloud.push_back(p);
    delete shape;
  }
  detected_shapes_.clear();

  return true;
}


ModelRec::PointCloudPtr 
ModelRec::loadPointCloudFromPointShape(PointSetShape * shape)
{
  PointCloudPtr new_cloud(new PointCloud());
  vtkPointSet * pointset = shape->getPolyData();
  for(vtkIdType id = 0; id < pointset->GetNumberOfPoints(); ++id)
    {
    double point_data[3];
    pointset->GetPoint(id, point_data);
    new_cloud->push_back(pcl::PointXYZ(point_data[0]/1000.0, point_data[1]/1000.0, point_data[2]/1000.0));
    }
  return new_cloud;
}
