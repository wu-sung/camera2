#include "rclcpp/rclcpp.hpp"  // ROS 2의 RCLCPP 라이브러리 포함
#include "sensor_msgs/msg/compressed_image.hpp"  // CompressedImage 메시지를 사용하기 위해 포함
#include "cv_bridge/cv_bridge.h"  // OpenCV와 ROS 메시지 간 변환을 위해 포함
#include "opencv2/opencv.hpp"  // OpenCV 라이브러리를 포함
#include <memory>
#include <chrono>

std::string src = "nvarguscamerasrc sensor-id=0 ! \
    video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
    width=(int)640, height=(int)360, format=(string)BGRx ! \
    videoconvert ! video/x-raw, format=(string)BGR ! appsink"; 

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);  // ROS 2 초기화
    auto node = std::make_shared<rclcpp::Node>("campub");  // 새로운 노드 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();  // QoS 설정
    auto mypub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile);
    // 퍼블리셔 생성(메세지 형태)
    
    std_msgs::msg::Header hdr;  // 메시지 헤더 생성
    sensor_msgs::msg::CompressedImage::SharedPtr msg;  // CompressedImage 메시지 포인터 생성
    rclcpp::WallRate loop_rate(40.0);  // 루프 속도 설정 (40Hz)

    cv::VideoCapture cap(src, cv::CAP_GSTREAMER);  // GStreamer를 사용하여 비디오 캡처 생성
    if (!cap.isOpened()) {  // 비디오 캡처 열기 실패 시
        RCLCPP_ERROR(node->get_logger(), "Video Missing!");  // 에러 로그 출력
        rclcpp::shutdown();  // ROS 2 종료
        return -1;  // 프로그램 종료
    }
    cv::Mat frame;  // 프레임을 저장할 변수

    while(rclcpp::ok())  // ROS 2가 실행 중일 때
    {
        cap >> frame;  // 비디오 캡처에서 프레임을 읽기
        if (frame.empty()) {  // 프레임이 비어 있으면
            RCLCPP_ERROR(node->get_logger(), "frame empty");  // 에러 로그 출력
            break;  // 루프 종료
        }
        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();  // 프레임을 CompressedImage 메시지로 변환
        mypub->publish(*msg);  // 메시지 퍼블리시
        loop_rate.sleep();  // 루프 속도 유지
    }
    rclcpp::shutdown();  // ROS 2 종료
    return 0;  // 프로그램 종료
}
