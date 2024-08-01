#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "count_book_cpp/srv/count_book.hpp"

int number = 0; 

/*
std::shared_ptr
 객체의 메모리 관리를 자동으로 처리 가능
 객체의 참조 카운트가 0이 되면 자동으로 메모리가 해제
 따라서, 메모리 누수 문제를 줄일 수 있음
*/

void count(const std::shared_ptr<count_book_cpp::srv::CountBook::Request> req,
            std::shared_ptr<count_book_cpp::srv::CountBook::Response> res)
{
  res->count_number = ++number;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), req->first_name << " " << req->last_name << " : "<< res->count_number);
}

int main(int args, char **argv)
{
  rclcpp::init(args, argv); // 노드 초기화 (ROS 2의 실행 환경을 설정하고, ROS 2 시스템과의 연결을 준비)

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("count_book_server"); 
  /* Node 객체를 동적으로 생성
  std::스마트_포인터<노드> 노드_변수명 = 노드::노드_객체_동적_생성("노드 이름");
  */

  rclcpp::Service<count_book_cpp::srv::CountBook>::SharedPtr service =
    node->create_service<count_book_cpp::srv::CountBook>("count_book_cpp", &count);
  /* service 서버 생성
  rclcpp::Service<count_book_cpp::srv::CountBook>::SharedPtr service = 
    서비스<서비스 타입>::스마트_포인터 서비스_변수명 =
  node->create_service<count_book_cpp::srv::CountBook>("count_book_cpp", &count);
    노드->서비스_생성<서비스 타입>("서비스 이름", &콜백함수);
  */ 

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"counting server activated");

  rclcpp::spin(node); // 노드의 이벤트 루프 시작
  rclcpp::shutdown(); // 해당 노드 종료
}