#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include "Eigen/Dense"

class KalmanFilter
{       
    public:
        KalmanFilter(){
            is_initialized_ = false ;
        };
        ~KalmanFilter();

        void Initialization(Eigen::VectorXd x_in){
            x_ = x_in ;
        }

        bool IsInitialized(){
            return this->is_initialized_ ;
        }

        /**********calculate / predict x'************/
        // state transistion matrix for predict x'
        void SetF(Eigen::MatrixXd F_in){
            F_ = F_in ;
        }

        /**********calculate / predict P'************/
        // state covariance matrix for predict x'
        void SetP(Eigen::MatrixXd P_in){
            P_ = P_in ;
        }
        // process covariance matrix for predict x'
        void SetQ(Eigen::MatrixXd Q_in){
            Q_ = Q_in ;
        }

        void SetH(Eigen::MatrixXd H_in){
            H_ = H_in ;
        }
        void SetR(Eigen::MatrixXd R_in){
            R_ = R_in ;
        }

        void Prediction(){
            x_ = F_ * x_ ;
            Eigen::MatrixXd Ft = F_.transpose() ;
            P_ = F_ * P_ * Ft + Q_ ;
        }

        void MeasurementUpdate(const Eigen::VectorXd &z){
            Eigen::VectorXd y = z - H_ * x_ ;
            Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_ ;
            Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse() ;
            x_ = x_ + (K * y) ;
            int size = x_.size() ;
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size,size) ;
            P_ = (I - K*H_) * P_ ;
        }

        Eigen::VectorXd GetX(){
            return x_ ;
        }

    private:
        bool is_initialized_ ;    // flag of initialized
        Eigen::VectorXd x_ ;      // state vector
        Eigen::MatrixXd F_ ;      // state transistion matrix
        Eigen::MatrixXd P_ ;      // state covariance matrix
        Eigen::MatrixXd Q_ ;      // process covariance matrix
        Eigen::MatrixXd H_ ;      // mesurement matrix
        Eigen::MatrixXd R_ ;      // mesurement covariance matrix

};
KalmanFilter::~KalmanFilter()
{
}

#endif 
