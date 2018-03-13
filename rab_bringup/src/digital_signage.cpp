#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class DigitalSignage{
 public:
   DigitalSignage(ros::NodeHandle &nh);
   int check_image();   
   
 private:
   int isInCircle(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, double x, double y, double radius);
   int isInAngle(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, double z, double w);
   void odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
   void publish_image(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

   int flag;   //画像を表示するためのフラグ
   double pose_x;
   double pose_y;
   
   // subscriber
   ros::Subscriber sub_odom;

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
   flag = 0;
   
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
   sub_odom = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(n.param<std::string>("odom_topic","odom"), 1, &DigitalSignage::odomCallback, this);
}

int DigitalSignage::isInCircle(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, double x, double y, double radius){
   double dist;
   dist = sqrt((fabs(msg->pose.pose.position.x)-fabs(x)) * (fabs(msg->pose.pose.position.x)-fabs(x))
	       + (fabs(msg->pose.pose.position.y)-fabs(y)) * (fabs(msg->pose.pose.position.y)-fabs(y)));
   ROS_WARN("dist : %lf", dist);
   ROS_WARN("radius : %lf", radius);
   if(dist < radius){
      return 0;
   }else{
      return 1;
   }
}

int DigitalSignage::isInAngle(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, double z, double w){
   // 角度の指定
}

void DigitalSignage::publish_image(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
   // 条件分岐
   if(isInCircle(msg, first_x_, first_y_, 1.0) == 0)
     flag = 1;
   else if(isInCircle(msg, second_x_, second_y_, 1.0) == 0)
     flag = 2;
   ROS_INFO("flag : %d", flag);
   
   // 画像表示
   if(count <= 0 && flag == 0){
      cv::imshow("Image", default_img);
      cv::waitKey(1);
   }else if(count == 0 && flag == 1){	 
      cv::imshow("Image", first_img);
      cv::waitKey(1);	  
   }else if(count == 0 && flag == 2){
      // 残り距離描画
      double dist = sqrt((fabs(msg->pose.pose.position.x)-fabs(second_x_)) * (fabs(msg->pose.pose.position.x)-fabs(second_x_))
	       + (fabs(msg->pose.pose.position.y)-fabs(second_y_)) * (fabs(msg->pose.pose.position.y)-fabs(second_y_)));
      char dist_c[256];
      sprintf(dist_c, "%lf", dist);
      cv::putText(second_img, dist_c, cv::Point(20,100), cv::FONT_HERSHEY_SIMPLEX, 10, cv::Scalar(0,0,0), 3, CV_AA);
		  cv::imshow("Image", second_img);
      cv::waitKey(1);
   }else{
      count+=1;
      if(count > 250)
	count = 0;
   }
   ROS_WARN("count = %d", count);
}

void DigitalSignage::odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
   /*
    double orientation_z = 0;
    double orientation_w = 0;
    double th0 = 0;
    th0 = 2 * asin(msg->pose.pose.orientation.z / 2);
    double th1 = 0;
    th1 = 2 * acos(msg->pose.pose.orientation.w / 2);
    ROS_INFO("Seq: [%d]", msg->header.seq);
    */ 
   ROS_INFO("Position-> x: [%lf], y: [%lf]", msg->pose.pose.position.x, msg->pose.pose.position.y);
   //ROS_INFO("Orientation-> th0: [%lf], th1: [%lf]", th0 * 180 / 3.14, th1 * 180 / 3.14);
   publish_image(msg);
}

int DigitalSignage::check_image(){
   // 画像が読み込まれなかったら終了
   if(first_img.empty() || second_img.empty() || default_img.empty()){
      ROS_ERROR("Load Image ERROR");
      return -1;
   }else{
      ROS_INFO("load Image success");
      return 0;
   }  
}  

int main(int argc, char **argv){
   ros::init(argc, argv, "digital_signage");
   ros::NodeHandle nh;
   DigitalSignage digital_signage(nh);
   if(digital_signage.check_image() == -1)
     return -1;

   // window作成
   cv::namedWindow("Image", CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
   cv::moveWindow("Image", 0, 0);

   ros::spin();
   return 0;
}