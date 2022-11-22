#include <memory>
#include <cstdio>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/impl/utils.h"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/msg/marker.hpp"

 
using std::placeholders::_1;

using Eigen::Matrix3d;
using Eigen::Vector3d;

using namespace std;


Vector3d u;
Vector3d y;
Vector3d X;

Matrix3d G_a = Matrix3d::Random();
Matrix3d G_b = Matrix3d::Random();
Matrix3d A = Matrix3d::Random();
Matrix3d C = Matrix3d::Random(); 

double eps = 10;
Matrix3d G_x = 1/(eps*eps) * Matrix3d::Random();

geometry_msgs::msg::PoseStamped robot_pose;



class QuadranListener : public rclcpp::Node {
    public : QuadranListener() : Node("quad_sub") {
        subscription_quad = this->create_subscription<geometry_msgs::msg::PoseStamped>("/quad/pose", 10, std::bind(&QuadranListener::quad_callback, this, _1));
    }

    private : 
        void quad_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const {
            tf2::Quaternion q_quad(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w); 
            X[0] = msg->pose.position.x; X[1] = msg->pose.position.y; X[2] = tf2::impl::getYaw(q_quad);

            robot_pose = *msg;

            RCLCPP_INFO(this->get_logger(), "I heard position: '%.2f', '%.2f', '%.2f'", X[0], X[1], X[2]);
        }

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_quad;
};


void Kalman_simple(Vector3d& X_pred, Matrix3d& G_pred, Vector3d u, Vector3d y, Matrix3d G_a, Matrix3d G_b, Matrix3d A, Matrix3d C) {
    Matrix3d S = C*G_pred*C.transpose() + G_b;
    Matrix3d K = G_pred*C.transpose()*S.inverse();
    Vector3d y_tilt = y - C*X_pred;
    Vector3d X_up = X_pred + K*y_tilt;
    Matrix3d G_up = (Matrix3d::Identity() - K*C) * G_pred;
    X_pred = A*X_up + u;
    G_pred = A*G_up*A.transpose() + G_a;
}

void lissajous(Vector3d& X, double t) {
    X << cos(t), 1./2.*sin(2*t), 0;
}



int main(int argc, char * argv[]) {
    rclcpp::init(argc,argv);
    rclcpp::Rate loop_rate(10);

    auto node = std::make_shared<QuadranListener>();
    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("/quad/simu", 1000);

    visualization_msgs::msg::Marker robot_marker;

    double t = 0;
    double dt = 0.01;
    u << 0, 0, 0;
    y << 0, 0, 0;

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        //SIMULATION
        robot_marker.header.frame_id = "map";
        robot_marker.header.stamp = QuadranListener().get_clock()->now();
        robot_marker.id = 0;
        robot_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        robot_marker.action = visualization_msgs::msg::Marker::ADD;
        robot_marker.pose = robot_pose.pose;

        robot_marker.scale.x = 1;
        robot_marker.scale.y = 1;
        robot_marker.scale.z = 1;
        robot_marker.color.a = 1.0; // alpha = transparence
        robot_marker.color.r = 1.0;
        robot_marker.color.g = 1.0;
        robot_marker.color.b = 1.0;
        robot_marker.mesh_resource = "package://Kalman/meshs/boat.dae";

        marker_pub->publish(robot_marker);


        //CONTROL
        lissajous(X, t);
        Kalman_simple(X, G_x, u, y, G_a, G_b, A, C);  
        t += dt;


        loop_rate.sleep();
    }
    return 0;
}

// class cam_sub : public rclcpp:Node {
//     public :
//         cam_sub() : Node("cam_sub") {
//             sub=this->create_subscription<cam_msg_type>("topic_cam",10,std::bind(&cam_sub::cam_callback,this,_1));
//         }

//     private : {
//         void cam_callback(const cam_msg_type &msg) const {
//             RCLCPP_INFO(this->get_logger(),"I heard: '%s'",msg.data.c_str());
//         }
//         rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
//     }
// }