#include "model_rec.h"



ModelRec::ModelRec(ros::NodeHandle* n, std::string pcl_pointcloud_channel, double pair_width, double voxel_size): n_(n), pcl_pointcloud_channel_(pcl_pointcloud_channel), objrec_(pair_width, voxel_size, 0.5), success_probability_(0.99) 
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
bool ModelRec::beginUpdatePCLPointCloud()
{

  cloud_queue.clear();
  pcl_point_cloud_.reset(new PointCloud);
  pcl_pointcloud_sub_ = n_->subscribe<PointCloud>(pcl_pointcloud_channel_, 1, &ModelRec::cloudQueuingCallback, this);
  return true;
}


void ModelRec::cloudQueuingCallback(const PointCloud::ConstPtr& msg)
{
  //probably take a mutex here 
  cloud_queue.push_back(msg);
  
  if (cloud_queue.size() == max_cloud_queue_size)
    {
      updatePCLPointCloud();
      pcl_pointcloud_sub_.shutdown();
    }
  
}

bool ModelRec::updatePCLPointCloud()
{
  BOOST_FOREACH(PointCloudConstPtr ptr , cloud_queue)
    {
      (*pcl_point_cloud_) += (*ptr);
    }
  return true;
}

bool ModelRec::updateVTKFromPCLCloud(){

  boost::lock_guard<boost::mutex> lock(ready_lock_);  
  pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PointXYZ> pgh(pcl_point_cloud_);
  pgh.getGeometry(vtk_cloud_ptr_); 
  vtk_point_cloud_ready_ = true;
  return true;
}


bool 
ModelRec::updateCloud()
{
  
  return updatePCLPointCloud() && updateVTKFromPCLCloud(); 
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
	    userData->setLabel("All");
	    // Load the model
	    sprintf(fileName, ("./data/" + s + ".vtk").c_str());
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
}

bool ModelRec::runRecognitionCallback(const ModelRec::FindObjects::Request & req, ModelRec::FindObjects::Response & res)
{
  ready_lock_.lock();
  vtk_point_cloud_ready_ = false;
  ready_lock_.unlock();
  beginUpdatePCLPointCloud();
  ros::Rate r(10);
  while(~vtk_point_cloud_ready_)
    {//spin
      r.sleep();
      ros::spinOnce();
    }
  removePlane();
  objrec_.doRecognition(foreground_vtk_cloud_ptr_, success_probability_, detected_shapes_);
}
