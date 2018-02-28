#include <ros/ros.h>
#include <std_msgs/Int8.h>

void flagCallback(const std_msgs::Int8::ConstPtr& msg){
   if(msg->data == 0){
      ROS_INFO("There is no obstacle.");
   }else if(msg->data == 1){
      ROS_ERROR("[left] within 1.0m");
   }else if(msg->data == 2){
      ROS_ERROR("[front] within 1.2m");
   }else if(msg->data == 3){
      ROS_ERROR("[right] within 1.0m");
   }else if(msg->data == 4){
      ROS_ERROR("[left][front] within 1.0m");
   }else if(msg->data == 5){
      ROS_ERROR("[front][right] within 1.0m");
   }else if(msg->data == 6){
      ROS_WARN("[left] within 1.5m");
   }else if(msg->data == 7){
      ROS_WARN("[front] within 1.7m");
   }else if(msg->data == 8){
      ROS_WARN("[right] within 1.5m");
   }else if(msg->data == 9){
      ROS_WARN("[left][front] within 1.5m");
   }else if(msg->data == 10){
      ROS_WARN("[front][right] within 1.5m");
   }else if(msg->data == 11){
      ROS_INFO("[left] within 2.0m");
   }else if(msg->data == 12){
      ROS_INFO("[front] within 2.0m");
   }else if(msg->data == 13){
      ROS_INFO("[right] within 2.0m");
   }else if(msg->data == 14){
      ROS_INFO("[left][front] within 2.0m");
   }else if(msg->data == 15){
      ROS_INFO("[front][right] within 2.0m");
   }
}

int main(int argc, char** argv){
   ros::init(argc, argv, "flag_subscriber");
   ros::NodeHandle n;
   ros::Subscriber sub_flag;
   sub_flag = n.subscribe("/flag", 10, flagCallback);
   ros::spin();
}
