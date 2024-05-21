#include "rclcpp/rclcpp.hpp"  // ROS 2 C++ 클라이언트 라이브러리 포함
#include "sensor_msgs/msg/compressed_image.hpp"  // 압축된 이미지 메시지 타입 포함
#include "opencv2/opencv.hpp"  // OpenCV 라이브러리 포함
#include "cv_bridge/cv_bridge.h"  // OpenCV와 ROS 메시지 간의 브릿지 포함
#include <memory>  // std::shared_ptr 사용을 위한 헤더 포함
#include <signal.h>  // 신호 처리를 위한 헤더 포함

cv::VideoWriter writer;  // VideoWriter 전역 변수 선언
bool ctrl_c_pressed = false;  // Ctrl+C 플래그

// Ctrl+C 핸들러 함수
void ctrlc_handler(int)
{
    ctrl_c_pressed = true;
}

// 콜백 함수: 메시지를 수신했을 때 호출됨
void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 압축된 이미지 메시지를 OpenCV 행렬로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (!frame.empty()) {  // 프레임이 비어 있지 않다면
        cv::imshow("Received Image", frame);  // 프레임을 창에 표시
        cv::waitKey(1);  // 키 입력 대기 (1밀리초)

        // 프레임을 동영상 파일에 저장
        writer.write(frame);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Empty frame");  // 오류 메시지 출력
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);  // ROS 2 초기화
    auto node = std::make_shared<rclcpp::Node>("camsub");  // 노드 생성

    // Ctrl+C 신호 핸들러 설정
    signal(SIGINT, ctrlc_handler);

    // VideoWriter 객체 초기화
    writer.open("output.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30, cv::Size(640, 360));
    if (!writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Could not open the output video file for write");
        rclcpp::shutdown();
        return -1;
    }

    // QoS 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // 서브스크립션 생성 및 콜백 함수 설정
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, mysub_callback);

    // 노드 스핀 (콜백 함수 실행)
    rclcpp::spin_some(node);

    // Ctrl+C 입력 시 종료
    while (rclcpp::ok() && !ctrl_c_pressed) {
        rclcpp::spin_some(node);
    }

    // 자원 정리
    writer.release();
    cv::destroyAllWindows();
    rclcpp::shutdown();  // ROS 2 종료
    return 0;
}
