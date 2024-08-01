#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "count_book/srv/count_book.hpp"

int number = 0;

void count(const std::shared_ptr<count_book::srv::CountBook::Request> req,
            std::shared_ptr<count_book::srv::CountBook::Response> res)
{
  res->count_number = ++number;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Incoming request::name:" << req->first_name << " " << req->last_name);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Count Number is %d", res->count_number);
}

int main(int args, char **argv)
{
  rclcpp::init(args, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("count_book_server");

  rclcpp::Service<count_book::srv::CountBook>::SharedPtr service =
    node->create_service<count_book::srv::CountBook>("count_book", &count);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"counting server activated");

  rclcpp::spin(node);
  rclcpp::shutdown();
}