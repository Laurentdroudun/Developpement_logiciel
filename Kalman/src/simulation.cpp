#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;


void lissajous(float_t &x, float_t &y, const float& t) {
	x = 10.* cos(t);
	y = 10. / 2. * sin(2 * t);
}

void dlissajous(float_t &x, float_t &y, const float& t){
	x = -10.*sin(t);
	y = 10.*cos(2 * t);
}


class Simulation : public rclcpp::Node {
	public:
		Simulation() : Node("simu") {
			publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/quad/pose", 10);
			timer_ = this->create_wall_timer(50ms, std::bind(&Simulation::timer_callback, this));
    	}

	private:
		float_t t = 0.;
		float_t dt = 0.05;
		float_t x, y, dx, dy;
		float_t theta;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;

		void timer_callback() {
			lissajous(x, y, t);
			dlissajous(dx, dy, t);

			auto q = tf2::Quaternion();
			theta = atan2(dy, dx);
			q.setRPY(0, 0, theta);
			q.normalize();

			auto simu_msg = geometry_msgs::msg::PoseStamped();
			simu_msg.header.stamp = Simulation().get_clock()->now();
			simu_msg.header.frame_id = "map";
      		simu_msg.pose.position.x = x;
			simu_msg.pose.position.y = y;
			simu_msg.pose.orientation = tf2::toMsg(q);

			RCLCPP_INFO(this->get_logger(), "q = [%.2f, %.2f, %.2f] ; t = %.2f", simu_msg.pose.position.x, simu_msg.pose.position.y, theta, t);
			publisher_->publish(simu_msg);
			t += dt;
    	}
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulation>());
  rclcpp::shutdown();
  return 0;
}