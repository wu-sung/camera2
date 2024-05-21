#include "rclcpp/rclcpp.hpp" // ROS 2 C++ 클라이언트 라이브러리
#include "sensor_msgs/msg/compressed_image.hpp" // 압축된 이미지 메시지 타입
#include "opencv2/opencv.hpp" // OpenCV 라이브러리
#include "cv_bridge/cv_bridge.h" // OpenCV와 ROS 메시지 간의 브릿지
#include <memory> // std::shared_ptr를 사용하기 위한 헤더
#include <functional> // std::bind를 사용하기 위한 헤더
#include <iostream> // 입출력 스트림 사용

using std::placeholders::_1; // std::bind에서 사용하기 위한 자리 표시자

// 콜백 함수: 메시지를 수신했을 때 호출됨
void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 압축된 이미지 메시지를 OpenCV 행렬로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (!frame.empty()) { // 프레임이 비어 있지 않다면
        cv::imshow("wsl", frame); // 프레임을 창에 표시
        cv::waitKey(10); // 키 입력 대기 (1밀리초)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received image: %s, %d x %d", msg->format.c_str(), frame.rows, frame.cols); // 수신된 프레임 정보 출력
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Empty frame~"); // 오류 메시지 출력
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); // ROS 2 초기화
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl"); // 노드 생성

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); // QoS 설정
    // 서브스크립션 생성 및 콜백 함수 설정
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, mysub_callback);

    rclcpp::spin(node); // 노드 스핀 (콜백 함수 실행)

    cv::destroyAllWindows(); // 모든 창 닫기
    rclcpp::shutdown(); // ROS 2 종료
    return 0; // 0값을 반환
}
