#include <iostream>
#include <Eigen/Dense>
 
using Eigen::Matrix3d;
using Eigen::Vector3d;

using namespace std;
 

void Kalman(Vector3d& X_pred, Matrix3d& G_pred, Vector3d u, Vector3d y, Matrix3d G_a, Matrix3d G_b, Matrix3d A, Matrix3d C) {
    Matrix3d S = C*G_pred*C.transpose() + G_b;
    Matrix3d K = G_pred*C.transpose()*S.inverse();
    Vector3d y_tilt = y - C*X_pred;
    Vector3d X_up = X_pred + K*y_tilt;
    Matrix3d G_up = (Matrix3d::Identity() - K*C) * G_pred;
    X_pred = A*X_up + u;
    G_pred = A*G_up*A.transpose() + G_a;
}


Vector3d f(Vector3d X, Vector3d u) {
    X = X+u-u;
    return X;
}



int main() {
    Vector3d u; u << 0.5, 2, 1;
    Vector3d y; y << 1, 1, 5;
    Matrix3d G_a = Matrix3d::Random();
    cout << G_a << endl;
    Matrix3d G_b = Matrix3d::Random();
    cout << G_b <<endl;
    Matrix3d A = Matrix3d::Random();
    Matrix3d C = Matrix3d::Random(); 

    Vector3d X0; X0 << 10, 5, 0;
    double eps = 10;
    Matrix3d G_x0 = 1/(eps*eps) * Matrix3d::Random();  

    Vector3d X = X0;
    Matrix3d G_x = G_x0;

    double dt = 0.01;

    for (double i=0; i<0.1; i+=dt) {
        X = X + f(X,u)*dt;
        Kalman(X, G_x, u, y, G_a, G_b, A, C);    
    }

    cout << X << endl;
    cout << G_x << endl;

    return 0;
}