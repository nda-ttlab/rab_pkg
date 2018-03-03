#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class DigitalSignage{
 public:
   DigitalSignage(ros::NodeHandle &nh);
   int check_image();   

 private:
   void waypointCallback(const visualization_msgs::Marker::ConstPtr& msg); 
   
   int flag;   //画像を表示するためのフラグ
   double pose_x;
   double pose_y;
   
   // subscriber
   ros::Subscriber sub_waypoint;

   // param for topic
   std::string first_path_;
   std::string second_path_;
   std::string default_path_;
   double first_x_;
   double first_y_;
   double second_x_;
   double second_y_;
   
   cv::Mat first_img;
   cv::Mat second_img;
   cv::Mat default_img;
   
   int count;

};
   
DigitalSignage::DigitalSignage(ros::NodeHandle &nh){
   // using parameter server
   ros::NodeHandle n("~");
   count = -10;
   
   // 画像読み込み1枚目(正門までの)
   n.param<std::string>("first_path", first_path_, ros::package::getPath("rab_bringup") + "/picture/sample.JPG");
   first_img = cv::imread(first_path_, 1);
   // 表示場所の設定
   n.param("first_x", first_x_, 0.0);
   n.param("first_y", first_y_, 0.0);
   
   // 2枚目(正門での案内)
   n.param<std::string>("second_path", second_path_, ros::package::getPath("rab_bringup") + "/picture/ubuntu-logo.png");
   second_img = cv::imread(second_path_, 1);
   // 表示場所の設定
   n.param("second_x", second_x_, 0.0);
   n.param("second_y", second_y_, 0.0);
   
   // デフォルト画像(カウントダウン)
   n.param<std::string>("default_path", default_path_, ros::package::getPath("rab_bringup") + "/picture/nabe.jpg");
   default_img = cv::imread(default_path_, 1);
   
   // subscribe topicの設定
   sub_waypoint = nh.subscribe<visualization_msgs::Marker>("waypoint_markers", 1, &DigitalSignage::waypointCallback, this);
}
 
/*
void DigitalSignage::publish_image(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
   // 画像表示
    
    
   
   if(isInCircle(msg, 0.0, 0.0, 1.0) == 0 || count == 0){
      flag = 0;
      cv::imshow("Image", default_img);
      cv::waitKey(1);
   }
   if(flag <= 10 && isInCircle(msg, first_x_, first_y_, 1.0) == 0){
      flag = 2;	 
      cv::imshow("Image", first_img);
      cv::waitKey(1);	  
      count = 0;
   }else if(flag <= 10 && isInCircle(msg, second_x_, second_y_, 2.0) == 0){
      flag = 3;	 
      cv::imshow("Image", second_img);
      cv::waitKey(1);	  
      count = 0;
   }else{
      flag = 1;
      count+=1;
      if(count > 500)
	count = ;
   }

   ROS_WARN("count = %d", count);
} */

void DigitalSignage::waypointCallback(const visualization_msgs::Marker::ConstPtr& msg){
   
   ROS_INFO("waypoint ID : %d", msg->id);
   //publish_image(msg);
}

int DigitalSignage::check_image(){
   // 画像が読み込まれなかったら終了
   if(first_img.empty() || second_img.empty() || default_img.empty()){
      ROS_ERROR("ERROR");
      return -1;	 
   }else{
      ROS_INFO("load success");
      return 0;
   }  
}

int main(int argc, char **argv){
   ros::init(argc, argv, "digital_signage");
   ros::NodeHandle nh;
   DigitalSignage digital_signage(nh);
/*   if(digital_signage.check_image() == -1){
	return -1;
   }
   
   // window作成
   cv::namedWindow("Image", CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
   cv::moveWindow("Image", 0, 0);
*/
   ros::spin();
   return 0;
}
