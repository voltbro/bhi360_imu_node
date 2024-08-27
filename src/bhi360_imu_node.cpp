#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <wchar.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "hidapi/hidapi.h"

#define MAX_STR 255

using namespace std::chrono_literals;
using namespace std;

class BHI360 : public rclcpp::Node
{
public:
  BHI360()
  : Node("bhi360_imu_node")
  {
    // ROS stuff
    this->declare_parameter("freq", 500.0);
    this->declare_parameter("imu_topic", "bhi360/imu");
    std::string imu_topic = this->get_parameter("imu_topic").get_parameter_value().get<std::string>();
    std::float_t freq = this->get_parameter("freq").get_parameter_value().get<std::float_t>();

    Ts = 1e0/freq;
    auto timer_period = Ts * 1000ms;

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
    timer_ = this->create_wall_timer(timer_period, std::bind(&BHI360::timer_callback, this));

    // imu stuff
    device_present = 1;
  
    if( open_imu_hid() )
    { 
      throw runtime_error("Unable to open IMU!");
    }

  }

  void timer_callback()
  {
    auto imu_msg = sensor_msgs::msg::Imu();

    uint8_t buffer[64];
	  int8_t bytes_number = 0;

    while(device_present)
    {
      bytes_number = hid_read(acc_handle, buffer, 64);
      
      if( bytes_number < 0 )
      {
        device_present = 0;
      }
      else if( bytes_number )
      {
        int16_t acc_x = *(int16_t*)&buffer[2];
        int16_t acc_y = *(int16_t*)&buffer[4];
        int16_t acc_z = *(int16_t*)&buffer[6];
        
        imu_msg.linear_acceleration.x = acc_x * 9.81f * 1.0f / 4096.0f;
        imu_msg.linear_acceleration.y = acc_y * 9.81f * 1.0f / 4096.0f;
        imu_msg.linear_acceleration.z = acc_z * 9.81f * 1.0f / 4096.0f;
        
        int16_t gyr_x = *(int16_t*)&buffer[8];
        int16_t gyr_y = *(int16_t*)&buffer[10];
        int16_t gyr_z = *(int16_t*)&buffer[12];
        
        imu_msg.angular_velocity.x = gyr_x * 6.28f * 2000.0f / 32768.0f / 360.0f;
        imu_msg.angular_velocity.y = gyr_y * 6.28f * 2000.0f / 32768.0f / 360.0f;
        imu_msg.angular_velocity.z = gyr_z * 6.28f * 2000.0f / 32768.0f / 360.0f;	
        
        int16_t quat_x = *(int16_t*)&buffer[14];
        int16_t quat_y = *(int16_t*)&buffer[16];
        int16_t quat_z = *(int16_t*)&buffer[18];
        int16_t quat_w = *(int16_t*)&buffer[20];
        
        imu_msg.orientation.x = quat_x * 1.0f / 16384.0f;
        imu_msg.orientation.y = quat_y * 1.0f / 16384.0f;
        imu_msg.orientation.z = quat_z * 1.0f / 16384.0f;
        imu_msg.orientation.w = quat_w * 1.0f / 16384.0f;
        
        imu_msg.header.stamp.nanosec = *(uint32_t*)&buffer[24] * 1000;
        
        break ;	
                
      }
      else
      {
        break ;
      }
    }

    if( !device_present )
    {
      throw runtime_error("IMU USB error!");
    }


    this->publisher_->publish(imu_msg);
  }

  int open_imu_hid()
  {
    int res;
    // unsigned char buf[65];
    wchar_t wstr[MAX_STR];

    // int i;

    // Initialize the hidapi library
    res = hid_init();
    
    struct hid_device_info *imu_info;
    imu_info = hid_enumerate(0xCafe, 0x4004);

    cout << "First interface path is " << imu_info->path << endl;

    acc_handle = hid_open_path(imu_info->path);
    if (!acc_handle) {
      cout << "Unable to open IMU!" << endl;
      hid_exit();
      return 1;
    }
    
    //hid_set_nonblocking(acc_handle, 1);

    cout << "IMU interface is opened successfully!" << endl;

    // Read the Manufacturer String
    res = hid_get_manufacturer_string(acc_handle, wstr, MAX_STR);
    cout << "Manufacturer String: " << wstr << endl;

    // Read the Product String
    res = hid_get_product_string(acc_handle, wstr, MAX_STR);
    cout << "Product String: " << wstr << endl;

    // Read the Serial Number String
    res = hid_get_serial_number_string(acc_handle, wstr, MAX_STR);
    cout << "Serial Number String: (" << wstr[0] << ") " << wstr << endl;
    
    return 0;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

  std::float_t Ts;
  uint8_t device_present;

  hid_device *acc_handle;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BHI360>());
  rclcpp::shutdown();
  return 0;
}