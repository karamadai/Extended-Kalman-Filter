#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}


void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    if(x_[0]==0 && x_[1]==0){
        x_[0]=0.1;
        x_[1]=0.1;
    }
    VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	//new estimate
	x_ = x_ + (K * y);
	P_ = (I_ - K * H_) * P_;
	//cout<<x_<<endl;


}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    VectorXd h_c2p=VectorXd(3);
    if(x_[0]==0) x_[0]=0.1;
    float h1=sqrt(x_[0]*x_[0]+x_[1]*x_[1]);
    float h2=atan(x_[1]/x_[0]);
    float h3=(x_[0]*x_[2]+x_[1]*x_[3])/h1;
    h_c2p<<h1,h2,h3; //Cartesian to polar transformer
    MatrixXd Hj = tools.CalculateJacobian(x_);
    VectorXd y = z-h_c2p;
    MatrixXd Ht = Hj.transpose();
	MatrixXd S = Hj * P_ * Ht + R_radar_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	//new estimate
	x_ = x_ + (K * y);
	P_ = (I_ - K * Hj) * P_;
}
