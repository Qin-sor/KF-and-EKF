#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H
#include "Eigen/Dense"
#include "matplotlibcpp.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <random>

class ExtendedKalmanFilter
{       
    public:
        ExtendedKalmanFilter(){
            is_initialized_ = false ;
        };
        ~ExtendedKalmanFilter();

        void Initialization(Eigen::VectorXd x_in){
            x_ = x_in ;
            is_initialized_ = true ; 
        }

        bool IsInitialized(){
            return this->is_initialized_ ;
        }

        /**********calculate / predict x'************/
        // state transistion matrix for predict x'
        void PredictX(double delta_time){
            // x_t prediction
            x_(0) = x_(0) + x_(3) * cos(x_(2)) * delta_time;
            x_(1) = x_(1) + x_(3) * sin(x_(2)) * delta_time;
            x_(2) = x_(2) + x_(4) * delta_time;
            //其余的值理论上不变
        }

        void Set_A(double delta_time){
            double theta = x_(2) ;
            double tmp_v = x_(3) ;
            double sv = -sin(theta)*tmp_v*delta_time ;
            double cv = cos(theta)*tmp_v*delta_time ;
            double st = sin(theta)*delta_time ;
            double ct = cos(theta)*delta_time ;
            A_ = Eigen::MatrixXd(5,5) ;
            //状态方程对状态向量求雅克比矩阵
            A_ << 1.0, 0.0, sv,  ct,  0.0,
                  0.0, 1.0, cv,  st,  0.0, 
                  0.0, 0.0, 1.0, 0.0, delta_time, 
                  0.0, 0.0, 0.0, 1.0, 0.0, 
                  0.0, 0.0, 0.0, 0.0, 1.0;      
        }
        void Set_P(Eigen::MatrixXd P_in){
            P_ = P_in ;
        }
        // state covariance matrix for predict x
        void PredictP(){
            P_ = A_ * P_ * A_.transpose() + Q_ ;        
        }

        // process covariance matrix for predict x'

        void SetQ(Eigen::MatrixXd Q_in){
            Q_ = Q_in ;
        }

        void SetH(){
            H_ = Eigen::MatrixXd(3,5) ;
            //观测方程对状态向量求雅可比矩阵
            H_ << 2*x_(0), 2*x_(1), 0,    0,   0,
                  0,       0,       0,    1,   0,
                  0,       0,       0,    0,   1;
        }

        void SetR(Eigen::MatrixXd R_in){
            R_ = R_in ;
        }

        void MeasurementUpdate(Eigen::MatrixXd z){
            Eigen::Vector3d z_pre (pow(x_(0),2) + pow(x_(1), 2), x_(3), x_(4)) ;
            Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_ ;
            Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse() ;
            x_ = x_ + (K * (z - z_pre)) ;  //观测-预测
            P_ = P_ - K*H_*P_ ;
        }

        Eigen::VectorXd GetX(){
            return x_ ;
        }

    private:
        bool is_initialized_ ;    // flag of initialized
        Eigen::VectorXd x_ ;      // state vector
        Eigen::MatrixXd A_ ;      // jacba matrix for P predict
        Eigen::MatrixXd P_ ;      // state covariance matrix
        Eigen::MatrixXd Q_ ;      // process covariance matrix
        Eigen::MatrixXd H_ ;      // mesurement matrix
        Eigen::MatrixXd R_ ;      // mesurement covariance matrix

};
ExtendedKalmanFilter::~ExtendedKalmanFilter()
{
}

#endif 
