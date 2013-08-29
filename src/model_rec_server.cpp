#include "model_rec.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv,"model_rec");
  ros::NodeHandle n;
  std::string cloud_topic = "/filtered_pc";
  n.getParam("cloud_topic", cloud_topic);
  std::cout << cloud_topic <<  "\n";
  //ModelRec m(&n, "/camera/depth_registered/points");
  ModelRec m(&n, cloud_topic);


  ros::Rate loop_rate(10);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;    
}
