#include <ros/ros.h>
#include <std_msgs/Int8.h>

void flagCallback(const std_msgs::Int8::ConstPtr& msg){
   if(msg->data == 0){
      ROS_INFO("There is no obstacle.");
   }else if(msg->data == 1){
      ROS_INFO("[right] There is a obstacle between 1.5m of 2m");
   }else if(msg->data == 2){
      ROS_WARN("[right] There is a obstacle between 1m of 1.5m");
   }else if(msg->data == 3){
      ROS_ERROR("[right] There is a obstacle within 1m");
   }else if(msg->data == 4){
      ROS_INFO("[left] There is a obstacle between 1.5m of 2m");
   }else if(msg->data == 5){
      ROS_WARN("[left] There is a obstacle between 1m of 1.5m");
   }else if(msg->data == 6){
      ROS_ERROR("[left] There is a obstacle within 1m");
   }else if(msg->data == 7){
      ROS_INFO("[front] There is a obstacle between 1.5m of 2m");
   }else if(msg->data == 8){
      ROS_WARN("[front] There is a obstacle between 1m of 1.5m");
   }else if(msg->data == 9){
      ROS_ERROR("[front] There is a obstacle within 1m");
   }
}

int main(int argc, char** argv){
   ros::init(argc, argv, "flag_subscriber");
   ros::NodeHandle n;
   ros::Subscriber sub_flag;
   sub_flag = n.subscribe("/flag", 10, flagCallback);
   ros::spin();
}
