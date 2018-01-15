#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class Digital_Signage{
 public:
   Digital_Signage(){
      ros::NodeHandle n("~");
      ros::Subscriber sub;
      count = 0;
      
      // window作成
      cv::namedWindow("Image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

      // 画像読み込み１枚目
      n.param<std::string>("first_path", first_path_, ros::package::getPath("rab_bringup") + "/picture/sample.JPG");
      first_img = cv::imread(first_path_, 1);
      // 表示場所の設定
      n.param("first_x", first_x_, 0.0);
      n.param("first_y", first_y_, 0.0);
      
      // 2枚目
      n.param<std::string>("second_path", second_path_, ros::package::getPath("rab_bringup") + "/picture/ubuntu-logo.png");
      second_img = cv::imread(second_path_, 1);
      // 表示場所の設定
      n.param("second_x", second_x_, 0.0);
      n.param("second_y", second_y_, 0.0);
      
      // デフォルト画像
      n.param<std::string>("default_path", default_path_, ros::package::getPath("rab_bringup") + "/picture/nabe.jpg");
      default_img = cv::imread(default_path_, 1);

      // subscribe topicの設定
      n.param<std::string>("odom_topic", odom_topic_, "/diff_drive_controller/odom");
      ROS_INFO("Subscribe topic : %s",odom_topic_.c_str());
      sub = n.subscribe("/diff_drive_controller/odom", 1000, &Digital_Signage::odomCallback, this);
      
      ROS_INFO("first_path = %s", first_path_.c_str());
      ROS_INFO("second_path = %s", second_path_.c_str());
      ROS_INFO("default_path = %s", default_path_.c_str());
      
      //publish_image();
      ros::spin();
   }
   
   void run(){
      while(ros::ok()){
	 ros::spin();
      }
   }
   
   void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){      
      //ROS_ERROR("hogehoge errorrrrrrrrrrrr");
      /*
       double orientation_z = 0;
       double orientation_w = 0;
       double th0 = 0;
       th0 = 2 * asin(msg->pose.pose.orientation.z / 2);
       double th1 = 0;
       th1 = 2 * acos(msg->pose.pose.orientation.w / 2);
       ROS_INFO("Seq: [%d]", msg->header.seq);
       */ 
      //ROS_INFO("Position-> x: [%lf], y: [%lf]", msg->pose.pose.position.x, msg->pose.pose.position.y);
      //ROS_INFO("Orientation-> th0: [%lf], th1: [%lf]", th0 * 180 / 3.14, th1 * 180 / 3.14);
      pose_x = msg->pose.pose.position.x;
      pose_y = msg->pose.pose.position.y;
      ROS_INFO("position = %lf, %lf", pose_x, pose_y);
      publish_image();
   }
   
   int check_image(){
      // 画像が読み込まれなかったら終了
      if(first_img.empty() || second_img.empty() || default_img.empty()) return -1;
      return 0;
   }
   
   void publish_image(){
      // window作成
      cv::namedWindow("Image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

      /*
      cv::imshow("Image", default_img);
      ROS_ERROR("start Digital Signage");
      cv::waitKey(1);
      */
      // 画像表示
      if(count > 100 || count == 0){
	 cv::imshow("Image", default_img);
	 ROS_ERROR("default img draw");
	 cv::waitKey(1);
      }
      if(flag == 0 && isInSquare(odom, 0.0, 0.0, 2.0) == 0){
	 flag = 1;	 
	 cv::imshow("Image", first_img);
	 ROS_ERROR("1st img draw");
	 cv::waitKey(1);	  
	 count = 0;
      }else if(flag == 0 && isInSquare(odom, 10.0, 10.0, 2.0) == 0){
	 flag = 1;	 
	 cv::imshow("Image", second_img);
	 ROS_ERROR("2nd img draw");
	 cv::waitKey(1);	  
	 count = 0;
      }else{
	 flag = 0;
	 count+=1;
	 ROS_WARN("count = %d", count);
      }
   }
   	 
   int isInSquare(const nav_msgs::Odometry::ConstPtr& msg, double x, double y, double meter){
      double current_x = msg->pose.pose.position.x;
      double current_y = msg->pose.pose.position.y;
      double plus_x = x + meter;
      double minus_x = x - meter;
      double plus_y = y + meter;
      double minus_y = y - meter;
      if(minus_x < current_x && current_x < plus_x && minus_y < current_y && current_y < plus_y){
	 return 0;
      }else{
	 return 1;
      }
   }   
   
   int flag;   //画像を表示するためのフラグ
   double pose_x;
   double pose_y;
   const nav_msgs::Odometry::ConstPtr odom;
   std::string odom_topic_;
   cv::Mat first_img;
   std::string first_path_;
   cv::Mat second_img;
   std::string second_path_;
   cv::Mat default_img;
   std::string default_path_;
   double first_x_;
   double first_y_;
   double second_x_;
   double second_y_;
   int count;

 private:

};

int main(int argc, char **argv){
   ros::init(argc, argv, "image_viewer");
   Digital_Signage digital_signage;
   int check_flag = digital_signage.check_image();
   if(check_flag == -1){
      ROS_ERROR("load Image ERROR!!");
      return -1;
   }
   digital_signage.run();
   return 0;
}
