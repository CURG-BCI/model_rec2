
import roslib
roslib.load_manifest( "model_rec2" )
import rospy
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size, hstack, vstack, mat, array, arange, fabs

import tf
import tf.transformations
import tf_conversions.posemath as pm
 
import model_rec2, model_rec2.srv
import sensor_msgs, sensor_msgs.msg
import pdb
import visualization_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import perception_msgs.msg

unique_model_name_postfix = 0

def get_unique_model_name_post_fix():
    unique_model_name_postfix += 1
    return unique_model_name_postfix


class Model( object ):
    def __init__(self, model_name, point_cloud_data, pose):
        self.model_name = model_name + get_unique_model_name_post_fix()
        self.object_name = model_name
        self.point_cloud_data = point_cloud_data
        self.pose = pose
        self.bc = ModelRecManager.tf_broadcaster
        self.listener = ModelRecManager.tf_listener
        
    def broadcast_tf(self):
        tf_pose = pm.toTf(pm.fromMsg(self.pose))
        self.bc.sendTransform(tf_pose[0], tf_pose[1], rospy.Time.now(), self.object_name, "/camera_rgb_optical_frame")
            
    def get_dist(self):
        self.broadcast_tf()
        self.listener.waitForTransform("/world", self.object_name, rospy.Time(0),rospy.Duration(10))
        (trans, rot) = self.listener.lookupTransform("/world",self.object_name, rospy.Time(0))
        return linalg.norm(trans)
    
    def get_world_pose(self):
        self.broadcast_tf()
        self.listener.waitForTransform("/world", self.object_name, rospy.Time(0),rospy.Duration(10))            
        return pm.toMsg(pm.fromTf(self.listener.lookupTransform("/world",self.object_name, rospy.Time(0))))

        



class ModelRecManager( object ):                  
            
    def __init__(self):
        if rospy.get_name() =='/unnamed':
            rospy.init_node('model_rec_node')

        self.model_list = list()
        self.segmented_objects_list = None

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
         
        self.segmented_objects_sub = rospy.Subscriber("segmented_objects", perception_msgs.msg.SegmentedObjectList, self.segmented_objects_list_callback )

        self.marker_array_pub = rospy.Publisher('/object_marker_array', visualization_msgs.msg.MarkerArray)
        self.object_pointcloud_pub = rospy.Publisher('/object_pointcloud',sensor_msgs.msg.PointCloud2) 
        
        self.find_objects_srv = rospy.ServiceProxy('/recognize_objects', model_rec2.srv.FindObjects)
        self.gen_hypotheses_srv = rospy.ServiceProxy('/gen_hypotheses', model_rec2.srv.GenHypotheses)
    
    #this is called whenever a new set of segmented objects is published
    def segmented_objects_list_callback(self, segmented_objects_list):
        self.segmented_objects_list = segmented_objects_list
        

    def run_model_rec(self):
        #clear old models
        self.model_list = list()

        #copy it so that we do not receive a new list of segmented objects
        #while running model_rec
        current_segmented_objects_list = self.segmented_objects_list

        #for each segmented object, find the matching models
        for segmented_object in current_segmented_objects_list.segmentedObjects:
            hypotheses = self.gen_hypotheses_srv(segmented_object.segmentedObjectPointCloud)
            found_object_resp = self.find_objects_srv(segmented_object.segmentedObjectPointCloud,hypotheses.model_hypotheses)

            #for each model found, add it to the model list
            for i in range(len(found_object_resp.object_name)):

                object_pose = found_object_resp.object_pose[i]
                object_name = found_object_resp.object_name[i]
                pointcloud = found_object_resp.pointcloud[i]

                model = Model(object_name,pointcloud,object_pose)

                self.model_list.append(model)


    def publish_closest_model(self):
        self.model_list.sort(key=Model.get_dist)
        closest_model = self.model_list[0]
        closest_model.broadcast_tf()
        closest_model.get_world_pose()
        self.object_pointcloud_pub.publish(closest_model.point_cloud_data)
        
print "call model_rec_manager = ModelRecManager()"
print "call model_rec_manager.run_model_rec() to run recognition"
print "call model_rec_manager.publish_closest_model() to publish best result" 





        

