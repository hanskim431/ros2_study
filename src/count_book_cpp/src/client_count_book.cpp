#include "rclcpp/rclcpp.hpp"
#include "count_book_cpp/srv/count_book.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  
  rclcpp::init(argc, argv); // 노드 초기화 (ROS 2의 실행 환경을 설정하고, ROS 2 시스템과의 연결을 준비)

  if(argc!=4) { // 명령줄 인수의 개수가 4개가 아닐 때
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: count_book first_name last_name phone_name");
    //get_logger 를 쓰기 위해서 rclcpp::init()을 먼저 수행
    return 1;
  } 

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("count_book_client");

  /* Node 객체를 동적으로 생성
    std::스마트_포인터<노드> 노드_변수명 = 노드::노드_객체_동적_생성("노드 이름");
  */

  rclcpp::Client<count_book_cpp::srv::CountBook>::SharedPtr client =
    node->create_client<count_book_cpp::srv::CountBook>("count_book_cpp");

  /* service 서버 생성
    rclcpp::Client<count_book_cpp::srv::CountBook>::SharedPtr client =
      클라이언트<클라이언트 타입>::스마트_포인터 클라이언트_변수명 =
    node->create_client<count_book_cpp::srv::CountBook>("count_book_cpp");
      노드->클라이언트_생성<클라이언트 타입>("클라이언트 이름", &콜백함수);
  */ 

  auto req = std::make_shared<count_book_cpp::srv::CountBook::Request>();
    req->first_name = argv[1];
    req->last_name = argv[2];
    req->phone_number = argv[3];

  /* 
  auto req = std::make_shared<count_book_cpp::srv::CountBook::Request>();
  자동_자료형 요청_변수명 = 스마트_포인터_생성<요청 타입>();
  count_book_cpp::srv::CountBook::Request 타입 객체 동적 생성
  */

  while (!client->wait_for_service(1s)) {
  /*
  wait_for_service(1s) => bool 타입
  true if the service is ready and the timeout is not over, false otherwise
    true => 서비스가 준비되고 터임아웃이 끝나지 않았을 때
  */

    if (!rclcpp::ok())
    {
    /*
    rclcpp::ok()
      true if shutdown was successful, false if context was already shutdown
      true => 종료가 성공적으로 이루어졌을 때
      false => 종료가 이미 이루어졌을 때
    */
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting...");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  
  auto result = client->async_send_request(req); // 비동기 요청 전송

  if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS){
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Your number is %d", result.get()->count_number);
  } else
  {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to call service count_book");
  }

  rclcpp::shutdown();
  return 0;
}
