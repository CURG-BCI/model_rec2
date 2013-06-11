#include "model_rec.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv,"model_rec");
  ros::NodeHandle n;
  //ModelRec m(&n, "/camera/depth_registered/points");
  ModelRec m(&n, "/filtered_pc");


  ros::Rate loop_rate(10);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;    
}
