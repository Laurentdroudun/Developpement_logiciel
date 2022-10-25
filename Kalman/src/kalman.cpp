#include <cstdio>
#include <rclcpp/rclcpp.hpp>


using namespace std;

class ins_sub : public rclpp:Node
{
  public :
  ins_sub() : Node("ins_sub")
  {
    sub=this->create_subscription<ins_msg_type>("topic",10,std::bind(&ins_sub::ins_callback,this,_1));
  }
  private :
  {
    void ins_callback(const ins_msg_type &msg) const {
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
  rclcpp::spin(std::make_shared<ins_sub>());
  rclcpp::shutdown();
  return 0;
}
