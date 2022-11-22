#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

//Heading resolution : 0.23°
//Acceleration resolution : 0.00981 m/s²

//inputs
// Vector3f y_k;
// Vector3f u_k;
// Matrix3f A_k;
// Matrix3f C_k;
// Matrix3f G_alpha_k;
// Matrix3f G_beta_k;

//auxiliary variables
// Vector3f x_k;
// Matrix3f G_k;
// Vector3f y_tilde_k;
// Matrix3f S_k;
// Matrix3f K_k;

//output
VectorXf x_kp1(3);
MatrixXf G_kp1(3,3);

void Kalman(Vector3f y_k,Vector3f u_k,Matrix3f A_k,Matrix3f C_k,Matrix3f G_alpha_k,Matrix3f G_beta_k) {
	Matrix3f K_k=G_kp1*C_k.transpose()*S_k.inverse();
	Matrix3f S_k=C_k*G_kp1*C_k.transpose()+G_beta_k;
	Vector3f y_tilde_k=y_k-C_k*x_kp1;
	Matrix3f G_k=(MatrixXd::Identity(3,3)-K_k*C_k)*G_kp1;
	Vector3f x_k=x_kp1+K_k*y_tilde_k;
	G_kp1=A_k*G_k*A_k.transpose()+G_alpha_k;
	x_kp1=A_k*x_k+u_k;
}

int main() {
	x_kp1 << 2,3,0;
	G_kp1 << 1,2,3,4,5,6,7,8,9;

	Matrix3f A_k=MatrixXd::Identity(3,3);
	Vector3f u_k={1,1,0};
	Matrix3f C_k=MatrixXd::Identity(3,3);
	Vector3f y_k={2,3,0};
	Matrix3f G_alpha_k=MatrixXd::Zeros(3,3);
	Matrix3f G_beta_k=MatrixXd::Zeros(3,3);
}