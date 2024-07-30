#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"

using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node {
public:AddressBookPublisher(): Node("address_book_publisher") {
    address_book_publisher_ = this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);

    auto publish_msg = [this]() -> void {
        auto message = more_interfaces::msg::AddressBook();

        /*
        ROS 2 빌드 시스템 (예: colcon build)은 이 .msg 파일을 읽고, 해당하는 C++ 코드를 생성합니다. 
        이 과정에서 메시지 타입에 대한 생성자, 접근자, 기타 유틸리티 함수가 자동으로 생성됩니다. 
        결과적으로, AddressBook 타입은 C++에서 사용할 수 있는 클래스가 되며, 
        이를 인스턴스화하기 위해 AddressBook() 생성자를 사용할 수 있게 됩니다.
        */

        message.first_name = "John";
        message.last_name = "Doe";
        message.phone_number = "1234567890";
        message.phone_type = message.PHONE_TYPE_MOBILE;

        std::cout << "Publishing Contact\nFirst:" << message.first_name <<
          "  Last:" << message.last_name << std::endl;

        this->address_book_publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookPublisher>());
  rclcpp::shutdown();

  return 0;
}