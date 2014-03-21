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
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/builtin_string.h>
#include <std_msgs/String.h>
#include <model_rec2/FindObjectsRequest.h>


void visualize(list<PointSetShape*>& detectedShapes, vtkPoints* scene, vtkPoints* background);

ModelRec::ModelRec(ros::NodeHandle* n, std::string pcl_pointcloud_channel, double pair_width, double voxel_size):
 n_(n),
 success_probability_(0.99) 
{
    srv_recognizeScene = n_->advertiseService("recognize_objects", &ModelRec::runRecognitionCallback, this);
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


void ModelRec::loadModels(ObjRecRANSAC objrec_)
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
			sprintf(fileName, (ros::package::getPath("model_rec2") + "/Recognition/data/" + s + ".vtk").c_str());
            vtkPolyDataReaderPtr reader(vtkPolyDataReader::New());
            reader->SetFileName(fileName);
            reader->Update();
            // Add the model to the model library
            vtkSmartPointer<vtkPolyData> model_data = reader->GetOutput();
            vtkSmartPointer<vtkPolyData> scaled_model_data = scale_vtk_model(model_data);
            ROS_INFO("hello%sWorld",fileName);

            //problem line
            objrec_.addModel(scaled_model_data, userData.get());

            // Save the user data and the reader in order to delete them later (outside this function)
            user_data_list_.push_back(userData);
            readers_.push_back(reader);
            scaled_shape_clouds_.push_back(scaled_model_data);
          }
}


void eigenFromCArray(const double *in, Eigen::Affine3d &out){
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


vtkSmartPointer<vtkPoints> ModelRec::convertPointCloudToVtk(sensor_msgs::PointCloud2 rosInputCloud)
{
    PointCloudPtr pclInputCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(rosInputCloud, *pclInputCloud);
    vtkSmartPointer<vtkPoints> vtkInputCloud = vtkSmartPointer<vtkPoints>::New();
    pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PointXYZ> pgh(pclInputCloud);
    pgh.getGeometry(vtkInputCloud);
    return vtkInputCloud;
}

bool ModelRec::runRecognitionCallback(model_rec2::FindObjects::Request & req, model_rec2::FindObjects::Response & res)
{
  ROS_INFO("Entering callback\n");

  ObjRecRANSAC *objrec_ = new ObjRecRANSAC(.015, .004, 0.5);
  objrec_->setVisibility(0.1);
  objrec_->setRelativeObjectSize(0.1);
  objrec_->setNumberOfThreads(4);

  model_list_.clear();

  vtkSmartPointer<vtkPoints> vtkInputCloud = convertPointCloudToVtk(req.search_cloud);
  for(int i = 0; i < req.model_hypotheses.size(); i++)
  {
      model_list_.push_back(req.model_hypotheses.at(i));
  }
  ROS_INFO("started load models\n");
  loadModels(*objrec_);
  ROS_INFO("finished load models\n");
  objrec_->doRecognition(vtkInputCloud, success_probability_, detected_shapes_);
  

//  ROS_INFO("Number of shapes: %i\n", detected_shapes_.size());

////  //process detected shapes and add them to response message
////  BOOST_FOREACH(PointSetShape * shape, detected_shapes_){

////    //Transform linear shape matrix to pose message
////    //FIXME this could definitely be more elegant
////    geometry_msgs::Pose shape_pose_msg;
////    Eigen::Affine3d shape_pose;
////    shape_pose.setIdentity();
////    const double *mat4x4 = shape->getRigidTransform();
    
////    //Stupid helper function to go from linear 4x4 to Eigen Affine3D
////    eigenFromCArray(mat4x4, shape_pose);
////    //Stupid helper to go from eigen to pose message
////    //FIXME: WARNING - this api is not the one that was documented
////    //it may be deprecated in favor of a different Eigen structure
////    //such as Eigen::Transform or Eigen2something
////    tf::poseEigenToMsg(shape_pose, shape_pose_msg);

////    //Print some basic information about detected shapes
////    std::cout << "Shape name " << shape->getUserData()->getLabel() << "\n"
////              << "Transform " << shape_pose_msg << "\n";

////    //store the detected shapes and poses.
////    //FIXME - These should be bundled together in a "localized_object" structure
////    //instead of implicitly matching them by index in their relative vectors.
////    res.object_name.push_back(std::string(shape->getUserData()->getLabel()));
////    res.object_pose.push_back(shape_pose_msg);

////    //Get complete point cloud for matched object
////    PointCloudPtr pc = loadPointCloudFromPointShape(shape);
////    pc->header.frame_id = shape->getUserData()->getLabel();
////    sensor_msgs::PointCloud2 p;
////    pcl::toROSMsg(*pc, p);
////    res.pointcloud.push_back(p);
////    delete shape;
////  }
//  ROS_INFO("DONE");

//  detected_shapes_.clear();
//  delete objrec_;
  return true;
}


ModelRec::PointCloudPtr ModelRec::loadPointCloudFromPointShape(PointSetShape * shape)
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



