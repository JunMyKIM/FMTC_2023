#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <ublox_msgs/ublox_msgs.h>
#include <ublox_msgs/NavPOSLLH.h>
#include <diagnostic_msgs/DiagnosticArray.h>




double Horizontal_accuracy = 0.0;
double Vertical_accuracy = 0.0;

ros::Publisher lane_switch_pub;
ros::Publisher gps_drive_mode_switch_pub;
//const diagnostic_msgs::DiagnosticArray::ConstPtr& msg
//const ublox_msgs::NavPOSLLH::ConstPtr& msg
void GpsDiagnosticCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg){
  for(const auto& status : msg->status){
    for(const auto& kv : status.values){
        // Horizontal Accuracy [m]의 값 찾기
        if(kv.key == "Horizontal Accuracy [m]"){
            // 값 출력
            std::cout << "Horizontal Accuracy: " << kv.value << std::endl;
            Horizontal_accuracy = std::stod(kv.value);
            // std::cout << "double 변환 : " << Horizontal_accuracy << std::endl;   
        }
        if(kv.key == "Vertical Accuracy [m]"){
            // 값 출력
            std::cout << "Vertical Accuracy: " << kv.value << std::endl;
            Vertical_accuracy = std::stod(kv.value);
            // std::cout << "double 변환 : " << Vertical_accuracy << std::endl;
            
        }
    }
  }



  if(Horizontal_accuracy > 0.05 || Vertical_accuracy > 0.05){

    std::cout<<"GPS 수신감도 이상 발생"<<std::endl;
    std::cout<<"차선인식 모드"<<std::endl;

    std_msgs::Int32 lane_flag;
    lane_flag.data = 1;// 1이면 lane_detection 모드 주행
    lane_switch_pub.publish(lane_flag);//lane_net 작동
    std::cout<<"Lane_switch_pub.data : "<<lane_flag.data<<std::endl;

    std_msgs::Int32 gps_diagnostic_flag;
    gps_diagnostic_flag.data = 1;// 1이면 GPS_drive OFF && lane_detection 모드 주행
    gps_drive_mode_switch_pub.publish(gps_diagnostic_flag);//lane_net 작동
    std::cout<<"gps_diagnostic_flag.data : "<<gps_diagnostic_flag.data<<std::endl;



  }

  else{

    std::cout<<"GPS 수신감도 정상"<<std::endl;
    std::cout<<"GPS 주행 모드"<<std::endl;


    std_msgs::Int32 lane_flag;
    lane_flag.data = 2;// 2이면 lane_detection OFF
    lane_switch_pub.publish(lane_flag);//lane_net OFF
    std::cout<<"Lane_switch_pub.data : "<<lane_flag.data<<std::endl;


    std_msgs::Int32 gps_diagnostic_flag;
    gps_diagnostic_flag.data = 2;// 2이면 GPS_drive ON && lane_detection OFF
    gps_drive_mode_switch_pub.publish(gps_diagnostic_flag);//GPS_drive ON
    std::cout<<"gps_diagnostic_flag.data : "<<gps_diagnostic_flag.data<<std::endl;    

  }

}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_gps_diagnostic");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/diagnostics", 1000, GpsDiagnosticCallback);
  lane_switch_pub = nh.advertise<std_msgs::Int32>("lane_switch", 1); 
  gps_drive_mode_switch_pub = nh.advertise<std_msgs::Int32>("gps_drive_mode_switch", 1); 

  ros::spin();

  return 0;
}
