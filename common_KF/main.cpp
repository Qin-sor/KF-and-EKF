#include "kalman_filter.hpp"
#include "matplotlibcpp.h"
#include <iostream>
#include <fstream>
#include <vector>

namespace plt = matplotlibcpp;

//read the data be created in ./data_script
bool GetLidarData(double& x, double& y, double& time){
    static bool is_read_file_finished = false ;
    static std::vector<double> file_x, file_y, file_time;
    static std::string line, value;

    if(!is_read_file_finished){
        is_read_file_finished = true ;
        std::string filename = "../data_script/file.txt";
        std::ifstream file(filename);

        while (std::getline(file, line)) {
                std::stringstream ss(line);
                std::getline(ss, value, ',');
                file_x.push_back(std::stod(value));
                std::getline(ss, value, ',');
                file_y.push_back(std::stod(value));
                std::getline(ss, value, ',');
                file_time.push_back(std::stod(value));
        }
    }
    if (file_time.size()>0){
                x = file_x.front() ;
                file_x.erase(file_x.begin()) ;

                y = file_y.front() ;
                file_y.erase(file_y.begin()) ;

                time = file_time.front() ;
                file_time.erase(file_time.begin()) ;
                return true ;
        } else {
                return false ;
        }  
}

int main(int argc, const char** argv) {
    double m_x = 0.0, m_y = 0.0 ;
    double last_timestamp = 0.0, now_timestamp = 0.0 ;
    KalmanFilter KF ;
    //for painting
    std::vector<double> x_origin(300), y_origin(300),x_now(300),y_now(300) ;
    while (GetLidarData(m_x, m_y, now_timestamp))
    // Initialize kalman filter
    {
        //////////for painting//////////
        x_origin.push_back(m_x) ;
        y_origin.push_back(m_y) ;
        //////////for painting//////////

        // Initialize P,Q,H,R
        if (!KF.IsInitialized())
        {
            last_timestamp = now_timestamp ;
            Eigen::VectorXd x_in(4,1) ;
            x_in << m_x, m_y, 0.0, 0.0 ;
            KF.Initialization(x_in) ;
            // state convarince matrix
            Eigen::MatrixXd P_in(4, 4) ;
            P_in << 1.0,   0.0,   0.0,   0.0, 
                    0.0,   1.0,   0.0,   0.0, 
                    0.0,   0.0,   100.0, 0.0, 
                    0.0,   0.0,   0.0,   100.0 ;

            KF.SetP(P_in) ;

            // process convariance matrix
            Eigen::MatrixXd Q_in(4, 4) ;
            Q_in << 1.0,  0.0,  0.0,  0.0, 
                    0.0,  1.0,  0.0,  0.0, 
                    0.0,  0.0,  1.0,  0.0, 
                    0.0,  0.0,  0.0,  1.0 ;
            KF.SetQ(Q_in) ;

            // measurement matrix
            Eigen::MatrixXd H_in(2, 4) ;
            H_in << 1.0, 0.0, 0.0, 0.0, 
                    0.0, 1.0, 0.0, 0.0 ;
            KF.SetH(H_in) ;

            // measurement convarince matrix
            //R is provided by Sensor producer, in their datasheet
            Eigen::MatrixXd R_in(2, 2) ;
            R_in << 0.0225, 0.0, 
                    0.0,    0.0225 ;
            KF.SetR(R_in) ;
        }
        
        //state transition matrix
        double delta_t = now_timestamp - last_timestamp ;
        last_timestamp = now_timestamp ;
        Eigen::MatrixXd F_in(4, 4) ;
        F_in << 1.0,    0.0,    delta_t, 0.0, 
                0.0,    1.0,    0.0,     delta_t, 
                0.0,    0.0,    1.0,     0.0, 
                0.0,    0.0,    0.0,     1.0 ;
        KF.SetF(F_in) ;

        KF.Prediction() ;
        // measurement value
        Eigen::VectorXd z(2, 1) ;
        z << m_x, m_y ;
        KF.MeasurementUpdate(z) ;

        //result
        Eigen::VectorXd x_out = KF.GetX() ;
        std::cout << "original output x : " << m_x <<
                                    " y: " << m_y << std::endl ;
        std::cout << "kalman output x : " << x_out(0) <<
                                    " y: " << x_out(1) << std::endl ;
        //////////for painting//////////
        x_now.push_back(x_out(0)) ;
        y_now.push_back(x_out(1)) ;
    }

    plt::figure_size(1200, 780);
    plt::scatter(x_origin, y_origin) ;

    plt::plot(x_now, y_now,"r");
    plt::title("raw data contrast to kfed data");
    plt::save("./basic.png");
    //////////for painting//////////
    return 0;
}