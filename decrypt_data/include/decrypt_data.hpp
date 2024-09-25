#pragma once

// C++
#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <time.h>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <algorithm>
#include <limits>
#include <random>
#include <condition_variable>
#include <mutex>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <opencv2/opencv.hpp>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <cryptopp/aes.h>
#include <cryptopp/filters.h>
#include <cryptopp/modes.h>
#include <cryptopp/osrng.h>
#include <cryptopp/hex.h>


#define _GUN_SOURCE

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

namespace Decrypt {

class Decryption : public rclcpp::Node
{
public:
  Decryption();
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ImageSubscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr DecryptedImagePublisher_;

  CryptoPP::SecByteBlock key;
  CryptoPP::SecByteBlock iv;
//  std::fill(key.begin(), key.end(), 'A');
//  std::fill(iv.begin(), iv.end(), 'A');

  bool tcp;
  int keylength;

  void initialize_aes();
  std::string decrypt(const std::string& plaintext);
  void DecryptionCallback(const sensor_msgs::msg::Image::SharedPtr msg);
private:

};

}
