#include <cstdio>
#include <rclcpp/rclcpp.hpp>


class quad_sub : public rclcpp:Node
{
  public :
  quad_sub() : Node("quad_sub")
  {
    sub=this->create_subscription<quad_msg_type>("topic_quad",10,std::bind(&quad_sub::quad_callback,this,_1));
  }
  private :
  {
    void quad_callback(const quad_msg_type &msg) const {
      RCLCPP_INFO(this->get_logger(),"I heard: '%s'",msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  }
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<quad_sub>());
  rclcpp::shutdown();
  return 0;
}

class cam_sub : public rclcpp:Node
{
  public :
  cam_sub() : Node("cam_sub")
  {
    sub=this->create_subscription<cam_msg_type>("topic_cam",10,std::bind(&cam_sub::cam_callback,this,_1));
  }
  private :
  {
    void cam_callback(const cam_msg_type &msg) const {
      RCLCPP_INFO(this->get_logger(),"I heard: '%s'",msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  }
}