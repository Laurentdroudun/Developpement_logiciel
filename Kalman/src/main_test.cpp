#include <iostream>
#include <Eigen/Dense>
 
using Eigen::Matrix3d;
using Eigen::Vector3d;

using namespace std;
 

void Kalman(Vector3d& X_pred, Matrix3d& G_pred, Vector3d u, Vector3d y, Matrix3d G_a, Matrix3d G_b, Matrix3d A, Matrix3d C, float alpha, Vector3d delta) {
    Matrix3d S = C*G_pred*C.transpose() + G_b;
    Matrix3d K = G_pred*C.transpose()*S.inverse();
    Vector3d y_tilt = y - C*X_pred;
    Vector3d X_up = X_pred + K*y_tilt;
    Matrix3d G_up = (Matrix3d::Identity() - K*C) * G_pred;
    X_pred = alpha*y+(1-alpha)*(A*X_up + u + delta);
    G_pred = A*G_up*A.transpose() + G_a;
}


void lissajous(Vector3d& X, double t) {
    X << cos(t), 1./2.*sin(2*t), 0;
}


int main() {
    Vector3d u; u << 2, 0, 0;
    Vector3d y; y << 1, 1, 5;
    Matrix3d G_a, G_b << pow(10,-1),0,0,0,pow(10,-1),0,0,0,pow(10,-1);
    Matrix3d G_b; G_b << pow(10,-1),0,0,0,pow(10,-1),0,0,0,pow(10,-1);
    Matrix3d A = Matrix3d::Identity();
    Matrix3d B = dt*Matrix3d::Identity();
    Matrix3d C = Matrix3d::Identity(); 

    double eps = 10;
    Matrix3d G_x0 = 1/(eps*eps) * Matrix3d::Random(); Matrix3d G_x = G_x0;

    Vector3d X;

    double dt = 0.05;

    for (double i=0; i<0.1; i+=dt) {
        lissajous(X, i);
        Kalman(X, G_x, u, y, G_a, G_b, A, C);    
    }

    cout << X << endl;
    cout << G_x << endl;

    return 0;
}