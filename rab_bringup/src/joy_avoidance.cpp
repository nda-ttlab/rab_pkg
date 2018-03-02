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
   int right_flag;
   int left_flag;
   int front_flag;
   int back_flag;
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
   right_flag = 0;
   left_flag = 0;
   front_flag = 0;
   flag = 0;
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
   // 位置に応じてフラグ変更(真後ろを0度として)
   this->right_front = scan->ranges.size() * 5 / 12; // 150度
   this->right_back = scan->ranges.size() * 1 / 12; // 30度
   this->left_front = scan->ranges.size() * 7 / 12; // 210度
   this->left_back = scan->ranges.size() * 11 / 12; // 330度
   double right_dist = minimumdist(scan, right_back, right_front, 0.0, 2.0);
   double front_dist = minimumdist(scan, right_front, left_front, 0.0, 3.0);
   double left_dist = minimumdist(scan, left_front, left_back, 0.0 ,2.0);
   double back_dist_1 = minimumdist(scan, left_back, 0, 0.0 ,2.0);
   double back_dist_2 = minimumdist(scan, 0, right_back, 0.0, 2.0);
   double back_dist;
   if(back_dist_1 < back_dist_2){
      back_dist = back_dist_1;
   }else if(back_dist_1 == back_dist_2){
      back_dist = 50.0;
   }else{
      back_dist = back_dist_2;
   }
   if(right_dist < 2.0){ // 右舷前方に障害物あり
      if(right_dist < 1.0){
	 right_flag = 3;
      }else if(right_dist < 1.5){
	 right_flag = 2;
      }else if(right_dist < 2.0){
	 right_flag = 1;
      }
   }else{
      right_flag = 0;
   }
   if(left_dist < 2.0){ // 左舷前方に障害物あり
      if(left_dist < 1.0){
	 left_flag = 3;
      }else if(left_dist < 1.5){
	 left_flag = 2;
      }else if(left_dist < 2.0){
	 left_flag = 1;
      }
   }else{
      left_flag = 0;
   }
   if(front_dist < 2.0){ // 正面に障害物あり
      if(front_dist < 1.2){
	 front_flag = 3;
      }else if(front_dist < 1.7){
	 front_flag = 2;
      }else if(front_dist < 2.0){
	 front_flag = 1;
      }
   }else{
      front_flag = 0;
   }
   if(back_dist < 2.0){  // 後方に障害物あり
      if(back_dist < 0.6){
	 back_flag = 3;
      }else if(back_dist < 1.2){
	 back_flag = 2;
      }else if(back_dist < 1.0){
	 back_flag = 1;
      }
   }else{
      back_flag = 0;
   }
	
   if(right_flag == 0 && left_flag == 0 && front_flag == 0 && back_flag == 0){
      // 障害物なし
      flag = 0;
   }else if(left_flag == 3 && front_flag != 3 && right_flag != 3){
      // 右1m以内に障害物
      flag = 1;
   }else if(left_flag != 3 && front_flag == 3 && right_flag != 3){
      // 正面1.2m以内に障害物
      flag = 2;
   }else if(left_flag != 3 && front_flag != 3 && right_flag == 3){
      // 左1m以内に障害物
      flag = 3;
   }else if(left_flag == 3 && front_flag == 3 && right_flag != 3){
      // 右1m,正面1m以内に障害物
      flag = 4;
   }else if(left_flag != 3 && front_flag == 3 && right_flag == 3){
      // 左1m,正面1.2以内に障害物
      flag = 5;
   }else if(left_flag == 2 && front_flag != 2 && right_flag != 2){
      // 右1.5m以内に障害物
      flag = 6;
   }else if(left_flag != 2 && front_flag == 2 && right_flag != 2){
      // 正面1.7m以内に障害物
      flag = 7;
   }else if(left_flag != 2 && front_flag != 2 && right_flag == 2){
      // 左1.5m以内に障害物
      flag = 8;
   }else if(left_flag == 2 && front_flag == 2 && right_flag != 2){
      // 右1.5m,正面1.7m以内に障害物
      flag = 9;
   }else if(left_flag != 2 && front_flag == 2 && right_flag == 2){
      // 左1.5m,正面1.7以内に障害物
      flag = 10;
   }else if(left_flag == 1 && front_flag != 1 && right_flag != 1){
      // 右2.0m以内に障害物
      flag = 11;
   }else if(left_flag != 1 && front_flag == 1 && right_flag != 1){
      // 正面2.0m以内に障害物
      flag = 12;
   }else if(left_flag != 1 && front_flag != 1 && right_flag == 1){
      // 左2.0m以内に障害物
      flag = 13;
   }else if(left_flag == 1 && front_flag == 1 && right_flag != 1){
      // 右2.0m,正面2.0m以内に障害物
      flag = 14;
   }else if(left_flag != 1 && front_flag == 1 && right_flag == 1){
      // 左2.0m,正面2.0以内に障害物
      flag = 15;
   }else if(back_flag == 3){
      // 後方1.2m以内に障害物
      flag = 16;
   }else if(back_flag == 2){
      // 後方1.7m以内に障害物
      flag = 17;
   }else if(back_flag == 1){
      // 後方2.0m以内に障害物
      flag = 18;
   }
   flag_states.data = flag;
}

void JoyAvoidance::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& vel){
   // フラグに応じてcmd_velの値を調整しをpublish
   geometry_msgs::Twist cmd_vel;
   // 右舷
   if(vel->angular.z < 0.0){
      if(right_flag == 1){
	 cmd_vel.angular.z = vel->angular.z * 0.8;
      }else if(right_flag == 2){
	 cmd_vel.angular.z = vel->angular.z * 0.5;
      }else if(right_flag == 3){
	 cmd_vel.angular.z = 0.0;
      }else{
	 cmd_vel.angular.z = vel->angular.z;
      }  
   }else if(vel->angular.z == 0.0){
      cmd_vel.angular.z = vel->angular.z;
   }else{ // 左舷
      if(left_flag == 1){
	 cmd_vel.angular.z = vel->angular.z * 0.8;
      }else if(left_flag == 2){
	 cmd_vel.angular.z = vel->angular.z * 0.5;
      }else if(left_flag == 3){
	 cmd_vel.angular.z = 0.0;
      }else{
	 cmd_vel.angular.z = vel->angular.z;
      }
   }
   // 正面
   if(vel->linear.x > 0.0){
      if(front_flag == 1){
	 cmd_vel.linear.x = vel->linear.x * 0.8;
      }else if(front_flag == 2){
	 cmd_vel.linear.x = vel->linear.x * 0.5;
      }else if(front_flag == 3){
	 cmd_vel.linear.x = 0.0;
      }else{
	 cmd_vel.linear.x = vel->linear.x;
      }
   }else if(vel->linear.x == 0.0){
      cmd_vel.linear.x = vel->linear.x;
   }else{
      if(back_flag == 1){
	 cmd_vel.linear.x = vel->linear.x * 0.8;
      }else if(back_flag == 2){
	 cmd_vel.linear.x = vel->linear.x * 0.5;
      }else if(back_flag == 3){
	 cmd_vel.linear.x = 0.0;
      }else{
	 cmd_vel.linear.x = vel->linear.x;
      }
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