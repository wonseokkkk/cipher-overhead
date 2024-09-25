#include "decrypt_data.hpp"

namespace Decrypt {

Decryption::Decryption()
       : Node("decryption_node", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true))
{
  this->get_parameter_or("key_size", keylength, 16);
  this->get_parameter_or("qos_tcp", tcp, false);
  key = CryptoPP::SecByteBlock(0x00, keylength);
  iv = CryptoPP::SecByteBlock(0x00, CryptoPP::AES::BLOCKSIZE);

  // Initialize AES context and set up encryption parameters
  initialize_aes();

  rclcpp::QoS default_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  default_qos.best_effort();
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  if(tcp){
    qos.reliable();
  }
  else qos.best_effort();

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  ImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>("encrypted_image", qos, std::bind(&Decryption::DecryptionCallback, this, std::placeholders::_1));


  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  DecryptedImagePublisher_ = this->create_publisher<sensor_msgs::msg::Image>("decrypted_image", default_qos);
}
  
void Decryption::initialize_aes()
{
//    CryptoPP::SecByteBlock key = CryptoPP::SecByteBlock(0x00, keylength);
//    CryptoPP::SecByteBlock iv = CryptoPP::SecByteBlock(0x00, CryptoPP::AES::BLOCKSIZE);

//    CryptoPP::AutoSeededRandomPool prng;
//    prng.GenerateBlock(key, key.size());
//    prng.GenerateBlock(iv, iv.size());
    std::fill(key.begin(), key.end(), 'A');
    std::fill(iv.begin(), iv.end(), 'A');

//    CryptoPP::SecByteBlock key = CryptoPP::SecByteBlock(0x00, CryptoPP::AES::DEFAULT_KEYLENGTH);
//    CryptoPP::SecByteBlock iv = CryptoPP::SecByteBlock(0x00, CryptoPP::AES::BLOCKSIZE);
}

std::string Decryption::decrypt(const std::string& ciphertext)
{
    std::string decryptedtext;
    CryptoPP::AES::Decryption aesDecryption(key, CryptoPP::AES::DEFAULT_KEYLENGTH);
    CryptoPP::CBC_Mode_ExternalCipher::Decryption cbcDecryption(aesDecryption, iv);
    CryptoPP::StreamTransformationFilter stfDecryptor(cbcDecryption, new CryptoPP::StringSink(decryptedtext));
    stfDecryptor.Put(reinterpret_cast<const unsigned char*>(ciphertext.c_str()), ciphertext.size());
    stfDecryptor.MessageEnd();
    
    if(decryptedtext != ""){
      RCLCPP_INFO(this->get_logger(), "Keysize = %i", keylength);
    }
    
    return decryptedtext;
    
}

void Decryption::DecryptionCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{

//    // 프레임 타이밍 계산
//    auto current_time = this->get_clock()->now();
//    if (last_frame_time_.nanoseconds() != 0) {
//        auto frame_duration = current_time - last_frame_time_;
//        double fps = 1.0 / frame_duration.seconds();
//        std::cout << "FPS: " << fps << std::endl;
//    }
//    last_frame_time_ = current_time;

    // 암호화된 데이터를 문자열로 변환
    std::string encrypted_msg(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());

    // 복호화
    std::string decrypted_msg = decrypt(encrypted_msg);

    // 암호화된 메시지 게시
    auto decrypted_image_msg = std::make_shared<sensor_msgs::msg::Image>(*msg);
    decrypted_image_msg->data.assign(decrypted_msg.begin(), decrypted_msg.end());

    DecryptedImagePublisher_->publish(*decrypted_image_msg);

//    // 복호화된 데이터를 OpenCV 이미지로 변환
//    cv_bridge::CvImagePtr cv_ptr;
//    try {
//        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//        std::memcpy(cv_ptr->image.data, decrypted_msg.data(), decrypted_msg.size());
//
//        // 이미지 출력
//        cv::imshow("Decrypted Image", cv_ptr->image);
//        cv::waitKey(1); // OpenCV 윈도우 업데이트
//    } catch (cv_bridge::Exception& e) {
//        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//        return;
//    }
}

} /* namespace */

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<Decrypt::Decryption>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

