#include "model_rec.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>

#include <pcl_ros/transforms.h>
#include <pcl/registration/transforms.h>
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>

#include <VtkBasics/VtkWindow.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkTransformPolyDataFilter.h>
#include <VtkBasics/VtkPolyData.h>
#include <VtkBasics/VtkPoints.h>
#include <vtkPoints.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include <ros/package.h>



void visualize(list<PointSetShape*>& detectedShapes, vtkPoints* scene, vtkPoints* background);

ModelRec::ModelRec(ros::NodeHandle* n, std::string pcl_pointcloud_channel, double pair_width, double voxel_size): n_(n), pcl_pointcloud_channel_(pcl_pointcloud_channel), max_cloud_queue_size(2), objrec_(pair_width, voxel_size, 0.5), success_probability_(0.99) 
{

  srv_recognizeScene = n_->advertiseService("recognize_objects", &ModelRec::runRecognitionCallback, this);

  model_list_.push_back("all");
  model_list_.push_back("garnier_shampoo_bottle");
  model_list_.push_back("gillette_shaving_gel");
  //model_list_.push_back("darpaflashlight");
  
  
  loadModels();
  objrec_.setVisibility(0.1);
  objrec_.setRelativeObjectSize(0.1);
  objrec_.setNumberOfThreads(4);  
}


/*@brief - Update internal pcl pointcloud by listening to point cloud stream
  
 */
bool ModelRec::beginUpdatePCLPointCloud()
{

  ROS_INFO("Entering begin update\n");
  foreground_vtk_cloud_ptr_ = vtkSmartPointer<vtkPoints>::New();
  background_vtk_cloud_ptr_ = vtkSmartPointer<vtkPoints>::New();
  cloud_queue.clear();
  //  objrec_.clear_rec();  
  pcl_point_cloud_.reset(new PointCloud);
  pcl_pointcloud_sub_ = n_->subscribe<PointCloud>(pcl_pointcloud_channel_, 1, &ModelRec::cloudQueuingCallback, this);
  return true;
}

/*@brief - Callback function to preprocess and save a point 
cloud from the point cloud stream for later processing. Unregister and 
call next stage in processing when the correct number of point clouds has been found. 

@param msg- Pointcloud message pointer from ROS.

This function puts valid point clouds in the point cloud queue. The purpose of
the point cloud queue is to super sample the scene. The number of samples
is determined by the max_cloud_size member.

FIXME: Test moving filtering functions out of this callback and in to the post capture
processing. 

 */

void ModelRec::cloudQueuingCallback(const PointCloud::ConstPtr& msg)
{

  PointCloudPtr cloud_NaN_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_outlier_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  ROS_INFO("queuing cloud\n");
  
  //Remove NaN from the cloud - it may be better to do this one the cloud is processed.
  std::vector<int> indeces;  //UNUSED
  pcl::removeNaNFromPointCloud (*msg, *cloud_NaN_filtered, indeces);

  //Remove statistical outliers from the cloud. This will remove some of the stuff under the table and such.

  /*This function looks at the standard deviation of the 50 nearest points. See PCL
  documentation for details. 
  Doing this after supersampling the point cloud will produce much more
  aggressive filtering because it overlays so many points. 
    */
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_NaN_filtered);
  sor.setMeanK (50);
  sor.setStddevMulThresh (3.0);
  sor.filter (*cloud_outlier_filtered);
  
  cloud_queue.push_back(cloud_outlier_filtered);

  if (cloud_queue.size() == max_cloud_queue_size)
    {
      pcl_pointcloud_sub_.shutdown();
      updateCloud();
    }
  
}

/*@brief - Concatenate stored pointclouds to make on big supersampled pointcloud of the scene. 

This function takes the pointcloud2 messages stored by previous stages and concatenates them.
It assumes that there are no incoming messages while it is working
Also assumes a previous function has cleared the current combined point cloud

FIXME - consider adding a lock here.

FIXME - Consider doing post processing steps here.
*/


bool ModelRec::updatePCLPointCloud()
{
  ROS_INFO("concatenating cloud\n");
  //Set new pointcloud header. Pointclouds with different headers cannot be
  //concatenated
  pcl_point_cloud_->header.frame_id=cloud_queue[0]->header.frame_id;
  int cloud_count = 0;

  //Loop through existing pointclouds and concatenate them
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

  pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PointXYZ> pgh(pcl_point_cloud_);
  pgh.getGeometry(vtk_cloud_ptr_); 
  vtk_point_cloud_ready_ = true;
  return true;
}


bool ModelRec::updateCloud()
{
    ROS_INFO("updating all clouds\n");
    updatePCLPointCloud();
     ROS_INFO("returned from update pcl cloud\n");
    updateVTKFromPCLCloud(); 
    ROS_INFO("returned from update vtk cloud\n");
    return true;
}

/*brief - helper function to scale vtk polydata meshes.

  Default use to convert from millimeters to meters
 */
vtkSmartPointer<vtkPolyData> scale_vtk_model(vtkSmartPointer<vtkPolyData> & m, double scale = 1.0/1000.0)
{
  vtkSmartPointer<vtkTransform> transp = vtkSmartPointer<vtkTransform>::New();
  transp->Scale(scale, scale, scale);
  vtkSmartPointer<vtkTransformPolyDataFilter> tpd = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
#if VTK_MAJOR_VERSION <= 5
  tpd->SetInput(m);
#else
  tpd->SetInputData(m);
#endif
  tpd->SetTransform(transp);
  tpd->Update();
  return tpd->GetOutput();
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
			//filename = ros::package::getPath(model_rec2);
            //sprintf(fileName, ("/home/armuser/ros/rosbuild_src/model_rec/Recognition/data/" + s + ".vtk").c_str());
			sprintf(fileName, (ros::package::getPath("model_rec2") + "/Recognition/data/" + s + ".vtk").c_str());
            vtkPolyDataReaderPtr reader(vtkPolyDataReader::New());
            reader->SetFileName(fileName);
            reader->Update();
            // Add the model to the model library
            vtkSmartPointer<vtkPolyData> model_data = reader->GetOutput();
            vtkSmartPointer<vtkPolyData> scaled_model_data = scale_vtk_model(model_data);
            objrec_.addModel(scaled_model_data, userData.get());
            // Save the user data and the reader in order to delete them later (outside this function)
            user_data_list_.push_back(userData);
            readers_.push_back(reader);
            scaled_shape_clouds_.push_back(scaled_model_data);
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
ModelRec::runRecognitionCallback(model_rec2::FindObjects::Request & req, model_rec2::FindObjects::Response & res)
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
  if(0){
    removePlane();
    objrec_.doRecognition(foreground_vtk_cloud_ptr_, success_probability_, detected_shapes_);
  }
  else{
    ROS_INFO("Number of points: %i\n", vtk_cloud_ptr_->GetNumberOfPoints());
    objrec_.doRecognition(vtk_cloud_ptr_, success_probability_, detected_shapes_);
  }

  
  // visualize point clouds
 #ifdef VISUALIZE_POINT_CLOUDS
  visualize(detected_shapes_, foreground_vtk_cloud_ptr_, background_vtk_cloud_ptr_);
 #endif


  ROS_INFO("Number of shapes: %i\n", detected_shapes_.size());

  //process detected shapes and add them to response message
  BOOST_FOREACH(PointSetShape * shape, detected_shapes_){

    //Transform linear shape matrix to pose message
    //FIXME this could definitely be more elegant
    geometry_msgs::Pose shape_pose_msg;
    Eigen::Affine3d shape_pose;
    shape_pose.setIdentity();
    const double *mat4x4 = shape->getRigidTransform();
    
    //Stupid helper function to go from linear 4x4 to Eigen Affine3D
    eigenFromCArray(mat4x4, shape_pose);
    //Stupid helper to go from eigen to pose message
    //FIXME: WARNING - this api is not the one that was documented
    //it may be deprecated in favor of a different Eigen structure
    //such as Eigen::Transform or Eigen2something
    tf::poseEigenToMsg(shape_pose, shape_pose_msg);

    //Print some basic information about detected shapes
    std::cout << "Shape name " << shape->getUserData()->getLabel() << "\n"
              << "Transform " << shape_pose_msg << "\n";

    //store the detected shapes and poses.
    //FIXME - These should be bundled together in a "localized_object" structure
    //instead of implicitly matching them by index in their relative vectors.
    res.object_name.push_back(std::string(shape->getUserData()->getLabel()));
    res.object_pose.push_back(shape_pose_msg);

    //Get complete point cloud for matched object
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
  vtkPointSet * pointset = shape->getHighResModel();
  for(vtkIdType id = 0; id < pointset->GetNumberOfPoints(); ++id)
    {
    double point_data[3];
    pointset->GetPoint(id, point_data);
    new_cloud->push_back(pcl::PointXYZ(point_data[0], point_data[1], point_data[2]));
    }
  return new_cloud;
}


void visualize(list<PointSetShape*>& detectedShapes, vtkPoints* scene, vtkPoints* background)
{
        printf("Visualizing ...\n");

        VtkWindow vtkwin(0, 0, 1000, 800);
          vtkwin.setCameraPosition(.131220071, -.240302073, -.162992888);
          vtkwin.setCameraFocalPoint(-.048026838, -.054679381, .787833180);
          vtkwin.setCameraViewUp(-0.044383, 0.978898, -0.199470);
          vtkwin.setCameraViewAngle(30.000000);
          vtkwin.setWindowSize(1000, 800);

        list<VtkPolyData*> transformedModelList;

        // Visualize the detected objects (important to look inside this loop)
        for ( list<PointSetShape*>::iterator it = detectedShapes.begin() ; it != detectedShapes.end() ; ++it )
        {
                PointSetShape* shape = (*it);
                // Which object do we have (and what confidence in the recognition result)
                if ( shape->getUserData() )
                        printf("\t%s, confidence: %lf\n", shape->getUserData()->getLabel(), shape->getConfidence());

                // Allocate memory for a homogeneous matrix
                double **mat4x4 = mat_alloc(4, 4);
                // Get the estimated rigid transform
                shape->getHomogeneousRigidTransform(mat4x4);

                const double *rigid_transform;
                rigid_transform = shape->getRigidTransform();

                std::cout << "Rotation Matrix: \n";
                for (int i = 0; i < 3; ++i){
                        for (int j = 0; j < 3; ++j)
                        {
                                std::cout << rigid_transform[i*3+j] << ' ';
                        }
                        std::cout << "\n";
                }
                std::cout << "Translation: \n";
                for (int i = 9; i < 12; ++i)
                        std::cout << rigid_transform[i] << ' ';
                std::cout << std::endl;


                // Transform the model instance using the estimated rigid transform
                vtkTransformPolyDataFilter *transformer = vtkTransformPolyDataFilter::New();
                  transformer->SetInput(shape->getHighResModel());
                  VtkTransform::mat4x4ToTransformer((const double**)mat4x4, transformer);

                // Visualize the transformed model
                VtkPolyData* transformedModel = new VtkPolyData(transformer->GetOutput());
                  transformedModel->setColor(1.0, 0.55, 0.05);
                  vtkwin.addToRenderer(transformedModel->getActor());
                  // Save in a list in order to delete outside this loop
                  transformedModelList.push_back(transformedModel);

                // Cleanup
                mat_dealloc(mat4x4, 4);
                transformer->Delete();
        }

        // Visualize the scene
        VtkPoints scenePoints(scene);
          scenePoints.selfAdjustPointRadius();
          scenePoints.setColor(0.1, 0.5, 1.0);
          vtkwin.addToRenderer(scenePoints.getActor());

        // Visualize the background
        VtkPoints* backgroundPoints = NULL;
        if ( background )
        {
                backgroundPoints = new VtkPoints(background);
                backgroundPoints->selfAdjustPointRadius();
                backgroundPoints->setColor(0.8, 0.8, 0.8);
                vtkwin.addToRenderer(backgroundPoints->getActor());
        }

        // The main vtk loop
        vtkwin.vtkMainLoop();

        // Cleanup
        for ( list<VtkPolyData*>::iterator it = transformedModelList.begin() ; it != transformedModelList.end() ; ++it )
                delete *it;
        if ( backgroundPoints )
                delete backgroundPoints;
}
