#include "encrypt_data.hpp"

namespace Encrypt {

Encryption::Encryption()
       : Node("encryption_node", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true))
{
  this->get_parameter_or("encryption/key_size", keylength, 16);
  this->get_parameter_or("encryption/qos_tcp", tcp, true);
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
  ImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/camera1/image_raw", default_qos, std::bind(&Encryption::EncryptionCallback, this, std::placeholders::_1));

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  EncryptedImagePublisher_ = this->create_publisher<sensor_msgs::msg::Image>("encrypted_image", qos);
}


void Encryption::initialize_aes()
{
//    CryptoPP::SecByteBlock key = CryptoPP::SecByteBlock(0x00, CryptoPP::AES::DEFAULT_KEYLENGTH);
//    CryptoPP::SecByteBlock iv = CryptoPP::SecByteBlock(0x00, CryptoPP::AES::BLOCKSIZE);
    //Create random key
//    CryptoPP::AutoSeededRandomPool prng;
//    prng.GenerateBlock(key, key.size());
//    prng.GenerateBlock(iv, iv.size());

    //fill A in key, iv
    std::fill(key.begin(), key.end(), 'A');
    std::fill(iv.begin(), iv.end(), 'A');


//    CryptoPP::SecByteBlock key = CryptoPP::SecByteBlock(0x00, CryptoPP::AES::DEFAULT_KEYLENGTH);
//    CryptoPP::SecByteBlock iv = CryptoPP::SecByteBlock(0x00, CryptoPP::AES::BLOCKSIZE);
}

std::string Encryption::encrypt(const std::string& plaintext)
{
    std::string ciphertext;
    CryptoPP::AES::Encryption aesEncryption(key, CryptoPP::AES::DEFAULT_KEYLENGTH);
    CryptoPP::CBC_Mode_ExternalCipher::Encryption cbcEncryption(aesEncryption, iv);
    CryptoPP::StreamTransformationFilter stfEncryptor(cbcEncryption, new CryptoPP::StringSink(ciphertext));
    stfEncryptor.Put(reinterpret_cast<const unsigned char*>(plaintext.c_str()), plaintext.length());
    stfEncryptor.MessageEnd();
    
    if(ciphertext != ""){
      RCLCPP_INFO(this->get_logger(), "Keysize = %i", keylength);
    }
    return ciphertext;

}

void Encryption::EncryptionCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 메시지 직렬화 (Serialization)
    std::string serialized_msg(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());

    // 암호화
    std::string encrypted_msg = encrypt(serialized_msg);

    // 암호화된 메시지 게시
    auto encrypted_image_msg = std::make_shared<sensor_msgs::msg::Image>(*msg);
    encrypted_image_msg->data.assign(encrypted_msg.begin(), encrypted_msg.end());

    EncryptedImagePublisher_->publish(*encrypted_image_msg);
}

} /* namespace */

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<Encrypt::Encryption>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

