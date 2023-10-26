#include "tello_driver_node.hpp"

namespace tello_driver
{
  void TelloSocket::listen()
  {
    thread_ = std::thread(
      [this]()
      {
        for (;;) {
          size_t r = socket_.receive(asio::buffer(buffer_));
          process_packet(r);
          // size_t r = socket_.receive_from(asio::buffer(buffer_), sender_endpoint_);
          // if(sender_endpoint_.address().to_string() == drone_ip_){
          //   std::cout << "drone ip: " << drone_ip_ << std::endl;
          //   process_packet(r);
          // }
        }
      });
  }

  bool TelloSocket::receiving()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return receiving_;
  }

  rclcpp::Time TelloSocket::receive_time()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return receive_time_;
  }

  void TelloSocket::timeout()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    receiving_ = false;
  }

} // namespace tello_driver
