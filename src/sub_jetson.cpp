#include "rclcpp/rclcpp.hpp"  // ROS 2의 RCLCPP 라이브러리를 포함
#include "sensor_msgs/msg/compressed_image.hpp"  // CompressedImage 메시지를 사용하기 위해 포함
#include "opencv2/opencv.hpp"  // OpenCV 라이브러리를 포함
#include "cv_bridge/cv_bridge.h"  // OpenCV와 ROS 메시지 간 변환을 위해 포함
#include <memory>
#include <functional>
#include <iostream>

using std::placeholders::_1;  // 바인딩을 위해 플레이스홀더 사용

cv::VideoWriter writer;  // 비디오 저장을 하기 위한 객체
bool is_writer_initialized = false;  // 비디오 초기화 여부
std::string output_file = "choi_video.mp4";  // 출력 파일 이름
int frame_width = 640;  // 프레임 너비
int frame_height = 360;  // 프레임 높이
int fps = 30;  // 초당 프레임 수

void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)  // 콜백 함수
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);  // CompressedImage 메시지를 OpenCV 이미지로 디코딩
    if (!frame.empty()) {  // 프레임이 비어 있지 않으면
        if (!is_writer_initialized) {  // 비디오가 초기화되지 않았으면
            writer.open(output_file, cv::VideoWriter::fourcc('M', 'P', '4', 'V'), fps, cv::Size(frame_width, frame_height), true);  // 비디오 작가 초기화
            if (!writer.isOpened()) {  // 비디오 열기 실패 시
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "error rorrr");  // 에러 로그 출력
                rclcpp::shutdown();  // ROS 2 종료
            }
            is_writer_initialized = true;  // 비디오 작가 초기화 완료
        }

        writer.write(frame);  // 프레임을 비디오 파일에 씀
        cv::imshow("jetson", frame);  // 프레임을 화면에 출력
        cv::waitKey(1);  // 키 입력 대기

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "received and saved frame: %s, %d x %d", msg->format.c_str(), frame.rows, frame.cols);  // 정보 로그 출력
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "empty frame received");  // 에러 로그 출력
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);  // ROS 2 초기화
    auto node = std::make_shared<rclcpp::Node>("camsub_save_and_display");  // 새로운 노드 생성

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();  // QoS 설정
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, mysub_callback);  // 서브스크립션 생성

    rclcpp::spin(node);  // 노드 실행

    if (is_writer_initialized) {  // 비디오가 초기화되었으면
        writer.release();  // 비디오 해제
    }
    cv::destroyAllWindows();  // 모든 OpenCV 창 닫기
    rclcpp::shutdown();  // ROS 2 종료
    return 0;
}
