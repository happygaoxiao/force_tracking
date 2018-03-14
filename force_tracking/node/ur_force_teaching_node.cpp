#include <ros/ros.h>
#include "../include/force_leader_follower/ur_force_teaching.hpp"


using teaching::URForceTeaching;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "URForceTeaching", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  std::string urdf_param;
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  std::string prefix;
  nh.param("ur_prefix", prefix, std::string(""));

  URForceTeaching forceTeaching(nh, urdf_param, prefix);


//    forceTeaching.learning_RPY();

    int order=0;
  while(order != 1&& order!=2)
  {
      std::cout<<"Press 1 to do gravity repair, press 2 to load existed repair file.\n"
               <<std::endl;
      std::cin>>order;
  }


    if(order==1) {
        forceTeaching.recordGravity();
        forceTeaching.record_force_sensor_repair();
        forceTeaching.load_force_sensor_repair();   //有标定文件的情况下使用，注意load的文件名
    }
    if(order==2) {
        forceTeaching.load_force_sensor_repair();   //有标定文件的情况下使用，注意load的文件名
    }


//    forceTeaching.test_servoj();       //test servoj tracking performance,   2.8mm position error
     forceTeaching.polishing_tracking();
//    forceTeaching.run();
}
