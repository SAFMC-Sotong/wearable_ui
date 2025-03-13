#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "mavros_msgs/msg/state.hpp"
#include <memory>
#include <chrono>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

using namespace std::chrono_literals;

class StreamData : public rclcpp::Node
{
public:
  StreamData() : Node("stream_data")
  {
    this->declare_parameter("forward_ip", "10.42.1.10");
    this->declare_parameter("battery_port", 7881);
    this->declare_parameter("mode_port", 7882);
    forward_ip_ = this->get_parameter("forward_ip").as_string();
    battery_port_ = this->get_parameter("battery_port").as_int();
    mode_port_ = this->get_parameter("mode_port").as_int();
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/mavros/battery_state", 10, std::bind(&StreamData::battery_callback, this, std::placeholders::_1));
    mode_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10, std::bind(&StreamData::mode_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(1ms, std::bind(&StreamData::timer_callback, this));
    udp_setup();
    RCLCPP_INFO(this->get_logger(), "Battery and state forwarding initialized");
  }
  ~StreamData()
  {
    if (battery_socket_ >= 0)
    {
      close(battery_socket_);
    }
    if (mode_socket_ >= 0)
    {
      close(mode_socket_);
    }
  }

private:
  void udp_setup()
  {
    battery_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (battery_socket_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create battery UDP socket");
      return;
    }

    memset(&battery_dest_addr_, 0, sizeof(battery_dest_addr_));
    battery_dest_addr_.sin_family = AF_INET;
    battery_dest_addr_.sin_port = htons(battery_port_);
    inet_pton(AF_INET, forward_ip_.c_str(), &battery_dest_addr_.sin_addr);

    mode_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (mode_socket_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create state UDP socket");
      return;
    }

    memset(&mode_dest_addr_, 0, sizeof(mode_dest_addr_));
    mode_dest_addr_.sin_family = AF_INET;
    mode_dest_addr_.sin_port = htons(mode_port_);
    inet_pton(AF_INET, forward_ip_.c_str(), &mode_dest_addr_.sin_addr);
  }
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    battery_msg_ = *msg;
    battery_update_ = true;
  }

  void mode_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    mode_msg_ = *msg;
    mode_update_ = true;
  }

  void udp_send()
  {
    if (battery_update_ && battery_socket_ >= 0)
    {
      std::string battery_msg = std::to_string(battery_msg_.percentage * 100.0f);
      if (sendto(battery_socket_, battery_msg.c_str(), battery_msg.length(), 0, (struct sockaddr*)&battery_dest_addr_,
                 sizeof(battery_dest_addr_)) < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to send battery data");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Battery: %.1f%%", battery_msg_.percentage * 100.0f);
        battery_update_ = false;  
      }
    }
    if (mode_update_ && mode_socket_ >= 0)
    {
    std::string mode_msg = format_state_message(mode_msg_);
      if (sendto(mode_socket_, mode_msg.c_str(), mode_msg.length(), 0, (struct sockaddr*)&mode_dest_addr_,
                 sizeof(mode_dest_addr_)) < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to send mode data");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Mode: %s", mode_msg_.mode.c_str());
        mode_update_ = false;
      }
    }
  }

  std::string format_state_message()
  { //format state message as "mode armed"
    std::string mode = mode_msg_.mode;
    std::string armed = mode_msg_.armed ? "armed" : "disarmed";
    return mode + " " + armed;
  }

  void timer_callback()
  {
    udp_send();
  }

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string forward_ip_;
  int battery_port_;
  int mode_port_;
  int battery_socket_;
  int mode_socket_;
  struct sockaddr_in battery_dest_addr_;
  struct sockaddr_in mode_dest_addr_;
  sensor_msgs::msg::BatteryState battery_msg_;
  mavros_msgs::msg::State mode_msg_;
  bool battery_update_ = false;
  bool mode_update_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StreamData>());
  rclcpp::shutdown();
  return 0;
}
