#include "rclcpp/rclcpp.hpp"  // ROS 2의 RCLCPP 라이브러리 포함
#include "sensor_msgs/msg/compressed_image.hpp"  // 압축된 이미지 메시지 타입 포함
#include "opencv2/opencv.hpp"  // OpenCV 라이브러리 포함
#include "cv_bridge/cv_bridge.h"  // OpenCV와 ROS 메시지 간 변환을 위한 브릿지 포함
#include <memory>  // std::shared_ptr 사용을 위한 헤더 포함
#include <functional>  // std::bind 사용을 위한 헤더 포함
#include <iostream>  // 입출력 스트림 사용

using std::placeholders::_1;  // std::bind에서 사용하기 위한 자리 표시자

cv::VideoWriter writer;  // 비디오 파일 작성을 위한 객체

void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 압축된 이미지 메시지를 OpenCV 행렬로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (!frame.empty()) {  // 프레임이 비어 있지 않다면
        cv::imshow("jetson", frame);  // 프레임을 창에 표시
        writer.write(frame);  // 프레임을 비디오 파일에 저장
        cv::waitKey(10);  // 키 입력 대기 (10밀리초)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received image: %s, %d x %d", msg->format.c_str(), frame.rows, frame.cols);  // 수신된 프레임 정보 출력
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Empty frame");  // 오류 메시지 출력
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);  // ROS 2 초기화
    auto node = std::make_shared<rclcpp::Node>("camsub_jetson");  // 새로운 노드 생성

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();  // QoS 설정
    // 서브스크립션 생성 및 콜백 함수 설정
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, mysub_callback);

    // 비디오 파일 작성을 위한 설정
    std::string filename = "output.mp4";  // 저장할 파일 이름
    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');  // 코덱 설정
    double fps = 30.0;  // 프레임 속도 설정
    cv::Size frame_size(640, 360);  // 프레임 크기 설정
    writer.open(filename, codec, fps, frame_size, true);  // 비디오 파일 열기

    if (!writer.isOpened()) {  // 비디오 파일 열기 실패 시
        RCLCPP_ERROR(node->get_logger(), "Could not open the output video file for write");  // 에러 로그 출력
        return -1;  // 프로그램 종료
    }

    rclcpp::spin(node);  // 노드 스핀 (콜백 함수 실행)

    writer.release();  // 비디오 파일 닫기
    cv::destroyAllWindows();  // 모든 창 닫기
    rclcpp::shutdown();  // ROS 2 종료
    return 0;  // 프로그램 종료
}
