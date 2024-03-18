#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <race/drive_values.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <cmath> // fmod 함수를 사용하기 위해 필요

ros::Publisher pose_pub;
ros::Publisher marker_pub;

double current_velocity = 0;

double current_velocity_1 = 0;
double input_steering = 0;
double input_throttle = 0;
double wheel_base = 2.5; // 차량의 휠베이스


double current_imu_heading;
double bicycle_heading;

geometry_msgs::Pose2D current_pose;

double normalize_angle(double angle) {
    // -360에서 360 범위로 정규화
    angle = std::fmod(angle, 360.0);
    // -180에서 180 범위로 조정
    if (angle > 180.0) {
        angle -= 360.0;
    } else if (angle < -180.0) {
        angle += 360.0;
    }
    return angle;
}

void imu_heading_callback(const std_msgs::Float64& msg){

  current_imu_heading = msg.data;


}

void speedCallback(const std_msgs::Float64& msg){

  current_velocity_1 = msg.data;


}



void control_values_callback(const race::drive_values::ConstPtr& msg)
{
  input_steering = msg->steering;
  input_throttle = current_velocity_1;

  double dt = 0.1; // 업데이트 시간 간격

  // 스로틀을 이용한 속도 업데이트 (가속도를 고려하여 계산)
  current_velocity += input_throttle * dt; // 간단한 예시, 실제 차량 모델에 따라 다를 수 있음

  // Bicycle 모델 계산
  double delta = atan(tan(input_steering) / wheel_base);
  current_pose.theta += current_velocity * dt * tan(delta) / wheel_base;
  current_pose.x += current_velocity * dt * cos(current_pose.theta);
  current_pose.y += current_velocity * dt * sin(current_pose.theta);

  bicycle_heading = normalize_angle(current_pose.theta);
  std::cout << "Heading : " << bicycle_heading << std::endl;

  std::cout << "Heading ERROR : " << abs(bicycle_heading-current_imu_heading) << std::endl;


  // visualization_msgs::Marker marker;
  // marker.header.frame_id = "map";
  // marker.header.stamp = ros::Time::now();
  // marker.ns = "bicycle_model";
  // marker.id = 0;
  // marker.type = visualization_msgs::Marker::ARROW;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.pose.position.x = current_pose.x;
  // marker.pose.position.y = current_pose.y;
  // marker.pose.position.z = 0;
  // marker.pose.orientation = tf::createQuaternionMsgFromYaw(current_pose.theta);
  // marker.scale.x = 1.0; // 화살표 길이
  // marker.scale.y = 0.1; // 화살표 폭
  // marker.scale.z = 0.1; // 화살표 높이
  // marker.color.a = 1.0; // Alpha는 1로 설정하여 완전히 불투명하게 설정
  // marker.color.r = 1.0;
  // marker.color.g = 0.0;
  // marker.color.b = 0.0;

  pose_pub.publish(current_pose);
  // marker_pub.publish(marker);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bicycle_model");
  ros::NodeHandle nh;

  pose_pub = nh.advertise<geometry_msgs::Pose2D>("vehicle_pose", 10);
  //marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Subscriber control_sub = nh.subscribe("control_value", 10, control_values_callback);
  ros::Subscriber Heading_sub = nh.subscribe("imu_Heading", 10, imu_heading_callback);
  ros::Subscriber speed_sub = nh.subscribe("current_speed", 100, speedCallback);
  ros::spin();
  return 0;
}
