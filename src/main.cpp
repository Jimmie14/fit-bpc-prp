#include <iostream>
#include "rclcpp/rclcpp.hpp"

class Receiver : public rclcpp::Node {
    public:
    Receiver() : Node("receiver") {

    }
};


int main() {
    std::cout << "Hello World!" << std::endl;
}