#include "rclcpp/rclcpp.hpp"  // ROS 2 C++ 클라이언트 라이브러리 포함
#include "sensor_msgs/msg/compressed_image.hpp"  // 압축된 이미지 메시지 타입 포함
#include "opencv2/opencv.hpp"  // OpenCV 라이브러리 포함
#include "cv_bridge/cv_bridge.h"  // OpenCV와 ROS 메시지 간의 브릿지 포함
#include <memory>  // std::shared_ptr 사용을 위한 헤더 포함
#include <functional>  // std::bind 사용을 위한 헤더 포함
#include <iostream>  // 입출력 스트림 사용
#include <signal.h>  // 신호 처리를 위한 헤더 포함

using std::placeholders::_1;  // std::bind에서 사용하기 위한 자리 표시자

bool ctrl_c_pressed = false;  // Ctrl+C가 눌렸는지 여부를 저장하는 변수

void ctrlc_handler(int) {
    ctrl_c_pressed = true;  // Ctrl+C가 눌리면 true로 설정
}

// 콜백 함수: 메시지를 수신했을 때 호출됨
void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg, cv::VideoWriter &writer)
{
    // 압축된 이미지 메시지를 OpenCV 행렬로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (!frame.empty()) {  // 프레임이 비어 있지 않다면
        cv::imshow("wsl", frame);  // 프레임을 창에 표시
        writer.write(frame);  // 프레임을 파일에 저장
        cv::waitKey(10);  // 키 입력 대기 (10밀리초)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received image: %s, %d x %d", msg->format.c_str(), frame.rows, frame.cols);  // 수신된 프레임 정보 출력
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Empty frame~");  // 오류 메시지 출력
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);  // ROS 2 초기화
    auto node = std::make_shared<rclcpp::Node>("camsub");  // 노드 생성
    signal(SIGINT, ctrlc_handler);  // Ctrl+C 신호 핸들러 설정

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();  // QoS 설정
    // 서브스크립션 생성 및 콜백 함수 설정
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile,
        std::bind(mysub_callback, _1, std::ref(writer))
    );

    // 비디오 파일 저장을 위한 VideoWriter 설정
    cv::VideoWriter writer("output.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30, cv::Size(640, 360));
    if (!writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Could not open the output video file for write");  // 오류 메시지 출력
        return -1;  // 프로그램 종료
    }

    while (rclcpp::ok() && !ctrl_c_pressed) {  // ROS 2가 실행 중이고, Ctrl+C가 눌리지 않았을 때
        rclcpp::spin_some(node);  // 콜백 함수 실행
        cv::waitKey(10);  // 키 입력 대기 (10밀리초)
    }

    cv::destroyAllWindows();  // 모든 창 닫기
    writer.release();  // 비디오 파일 저장 종료
    rclcpp::shutdown();  // ROS 2 종료
    return 0;  // 프로그램 종료
}
