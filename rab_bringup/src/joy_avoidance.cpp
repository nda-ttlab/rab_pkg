/* joy_control.launchからpublishされるcmd_velのデータをsubscribe、
   base_scanもsubscribeし、進みたい方向に障害物があればcmd_velを0にし、その状況に応じたフラグをpublish */

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

class JoyAvoidance{
 public:
   JoyAvoidance(ros::NodeHandle &nh);
   double minimumdist(const sensor_msgs::LaserScan::ConstPtr& scan, 
		double angle_1, double angle_2, double min_dist, double max_dist);
   
 private:
   void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
   void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& vel);
   
   // Subscrier
   ros::Subscriber sub_scan;
   ros::Subscriber sub_vel;
   // Publisher
   ros::Publisher pub_vel;
   ros::Publisher pub_flag;
   
   // declaration of menber variable
   int right_front;
   int right_back;
   int left_front;
   int left_back;
   int flag;
   std_msgs::Int8 flag_states;
};

JoyAvoidance::JoyAvoidance(ros::NodeHandle &nh){
   ros::NodeHandle n("~");
   // Subscriber
   sub_scan = nh.subscribe<sensor_msgs::LaserScan>(n.param<std::string>("scan_topic","base_scan"), 1, &JoyAvoidance::sensorCallback, this);
   sub_vel = nh.subscribe<geometry_msgs::Twist>(n.param<std::string>("sub_cmdvel_topic","cmd_vel_old"), 1, &JoyAvoidance::cmdvelCallback, this);
   // Publisher
   pub_vel = nh.advertise<geometry_msgs::Twist>(n.param<std::string>("pub_cmdvel_topic","cmd_vel"), 1);
   pub_flag = nh.advertise<std_msgs::Int8>("/flag", 10);
}

double JoyAvoidance::minimumdist(const sensor_msgs::LaserScan::ConstPtr& scan, 
				 double angle_1, double angle_2, double min_dist, double max_dist){
   double distance = 50.0;
   for(int i = angle_1; i <= angle_2; i++){
      if(min_dist < scan->ranges[i] && scan->ranges[i] < max_dist){
	 distance = scan->ranges[i];
      }
   }
   if(distance != 50.0){
      return distance;
   }else{
      return 50.0;
   }
}

void JoyAvoidance::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
   // 位置に応じてフラグ変更
   this->right_front = scan->ranges.size() * 3 / 8; // 右45度前方
   this->right_back = scan->ranges.size() * 1 / 8; // 右45度後方
   this->left_front = scan->ranges.size() * 5 / 8; // 左45度前方
   this->left_back = scan->ranges.size() * 7 / 8; // 左45度後方
   double right_dist = minimumdist(scan, right_back, right_front, 0.0, 2.0);
   double front_dist = minimumdist(scan, right_front, left_front, 0.0, 3.0);
   double left_dist = minimumdist(scan, left_front, left_back, 0.0 ,2.0);
   int flag_angle;
   if(right_dist < front_dist && right_dist < left_dist){
      flag_angle = 1; // 右舷前方に障害物あり
   }else if(left_dist < front_dist && left_dist < right_dist){
      flag_angle = 2; // 左舷前方に障害物あり
   }else if(front_dist < right_dist && front_dist < left_dist){
      flag_angle = 3; // 正面に障害物あり
   }else{
      flag_angle = 0; // 障害物なし
   }
   switch(flag_angle){
    case 1:
      if(right_dist < 1.0){
	 flag = 3;
	 break;
      }else if(right_dist < 1.5){
	 flag = 2;
	 break;
      }else if(right_dist < 2.0){
	 flag = 1;
	 break;
      }
    case 2:
      if(left_dist < 1.0){
	 flag = 6;
	 break;
      }else if(left_dist < 1.5){
	 flag = 5;
	 break;
      }else if(left_dist < 2.0){
	 flag = 4;
	 break;
      }
    case 3:
      if(front_dist < 1.5){
	 flag = 9;
	 break;
      }else if(front_dist < 2.5){
	 flag = 8;
	 break;
      }else if(front_dist < 3.0){
	 flag = 7;
	 break;
      }      
    case 0:
      flag = 0;
      break;
   }
   flag_states.data = flag;
}

void JoyAvoidance::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& vel){
   // フラグに応じてcmd_velの値を調整しをpublish
   geometry_msgs::Twist cmd_vel;
   // 右舷前方
   if(flag == 1 && (vel->linear.z < 0.0)){
      cmd_vel.angular.z = vel->angular.z * 0.8;
   }else if(flag == 2 && (vel->linear.z < 0.0)){
      cmd_vel.angular.z = vel->angular.z * 0.5;
   }else if(flag == 3 && (vel->linear.z < 0.0)){
      cmd_vel.angular.z = 0.0;
   }
   // 左舷前方
   if(flag == 4 && (vel->linear.z > 0.0)){
      cmd_vel.angular.z = vel->angular.z * 0.8;
   }else if(flag == 5 && (vel->linear.z > 0.0)){
      cmd_vel.angular.z = vel->angular.z * 0.5;
   }else if(flag == 6 && (vel->linear.z > 0.0)){
      cmd_vel.angular.z = 0.0;
   }
   // 正面
   if(flag == 7 && (vel->linear.x > 0.0)){
      cmd_vel.linear.x = vel->linear.x * 0.8;
   }else if(flag == 8 && (vel->linear.x > 0.0)){
      cmd_vel.linear.x = vel->linear.x * 0.5;
   }else if(flag == 9 && (vel->linear.x > 0.0)){
      cmd_vel.linear.x = 0.0;
   }else{
      cmd_vel.linear.x = vel->linear.x;
      cmd_vel.angular.z = vel->angular.z;
   }
   cmd_vel.linear.y = 0.0;
   pub_vel.publish(cmd_vel);
   pub_flag.publish(flag_states);
}

int main(int argc, char** argv){
   ros::init(argc, argv, "joy_avoidance");
   ros::NodeHandle n;
   JoyAvoidance joy_avoidance(n);
   ros::spin();
}