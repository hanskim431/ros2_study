#include "rclcpp/rclcpp.hpp"
#include "count_book/srv/count_book.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  
  rclcpp::init(argc, argv);

  if(argc!=4) { // 명령줄 인수의 개수가 4개가 아닐 때
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: count_book first_name last_name phone_name");
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("count_book_client");

  rclcpp::Client<count_book::srv::CountBook>::SharedPtr client =
    node->create_client<count_book::srv::CountBook>("count_book");

  auto req = std::make_shared<count_book::srv::CountBook::Request>();
    req->first_name = argv[1];
    req->last_name = argv[2];
    req->phone_number = argv[3];

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting...");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  
  auto result = client->async_send_request(req);

  if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS){
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Your number is %d", result.get()->count_number);
  } else
  {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to call service count_book");
  }

  rclcpp::shutdown();
  return 0;
}
