#include "extended_kalman_filter.hpp"

namespace plt = matplotlibcpp;

// creat data
void create_data(std::vector<Eigen::VectorXd> &x_stand, 
                std::vector<Eigen::VectorXd> &x_noise, 
                std::vector<Eigen::VectorXd> &measure, 
                int all_data)
{
    double car_speed = 0.1, delta_time = 0.1, car_omega = 0.01;
    bool watch = false ;
    double tmp_x = 0, tmp_y = 0, tmp_theta = 0, tmp_v = 0, tmp_w = 0; //初值0

    std::random_device rd;                             // 用于生成种子
    std::mt19937 random_w(rd());                       // 以随机设备生成的种子初始化Mersenne Twister伪随机数生成器
    std::uniform_real_distribution<> dis(0, 0.01); // 定义一个0-0.01之间的均匀分布

    // inital value
    // first value
    Eigen::VectorXd a(5, 1), a_measure(3, 1) ;
    a << tmp_x, tmp_y, tmp_theta, tmp_v, tmp_w;
    x_stand.push_back(a);
    x_noise = x_stand;
    a_measure << 0.0, 0.0, 0.0 ;
    measure.push_back(a_measure) ;

    for (size_t i = 0; i < all_data; i++)
    { // standard state data
      // x1 = x0 + v*cos(theta)*time ;   v的x分量速度
        tmp_x = x_stand[i](0) + car_speed * cos(x_stand[i](2)) * delta_time;
        tmp_y = x_stand[i](1) + car_speed * sin(x_stand[i](2)) * delta_time;
        tmp_theta = x_stand[i](2) + car_omega * delta_time;
        a << tmp_x, tmp_y, tmp_theta, car_speed, car_omega;
        x_stand.push_back(a);
      // noise measure data
        float speed_noise = car_speed + dis(random_w) * 10; // 原速度0.1 + 一个0-0.1的值
        float omega_noise = car_omega + dis(random_w);
        a_measure(0) = pow(tmp_x, 2) + pow(tmp_y, 2) + dis(random_w)/10 ;
        a_measure(1) = speed_noise ;
        a_measure(2) = omega_noise ;
        measure.push_back(a_measure) ;
  
        // noise state data
        speed_noise = car_speed + dis(random_w) * 10; // 原速度0.1 + 一个0-0.1的值
        omega_noise = car_omega + dis(random_w);

        tmp_x = x_noise[i](0) + speed_noise * cos(x_noise[i](2)) * delta_time;
        tmp_y = x_noise[i](1) + speed_noise * sin(x_noise[i](2)) * delta_time;
        tmp_theta = x_stand[i](2) + omega_noise * delta_time;
        a << tmp_x, tmp_y, tmp_theta, speed_noise, omega_noise;
        x_noise.push_back(a);
    }
    if (watch)
    {
        std::cout << std::fixed; // 输出固定格式
        std::cout.precision(4); // 保留4位小数
        for (size_t i = 0; i < all_data; i++)
        {
            for (size_t j = 0; j < 5; j++)
            {
                    std::cout <<  x_noise[i](j) << "  ";
            }
            std::cout << std::endl ;
        }
    }
    
}

int main(int argc, const char **argv)
{
    int all_data_num = 1000;
    double delta_time = 0.1 ;  //two state vector(1, 5) and one measure vector(1,3)
    std::vector<Eigen::VectorXd> x_stand, x_noise, measure_x; 
    static std::vector<double> vis_x_std, vis_y_std, vis_x_nse, vis_y_nse, vis_x_ekf, vis_y_ekf;
    create_data(x_stand, x_noise, measure_x, all_data_num);

    ExtendedKalmanFilter EKF;
    while (x_noise.size())
    {
        // Initialize kalman filter
        if (!EKF.IsInitialized())
        {
            Eigen::VectorXd x_in(x_noise[0]) ;
            EKF.Initialization(x_in) ;

            Eigen::MatrixXd P_in(5, 5) ;
            P_in << 10,  10,   1,    0,           0, 
                    10,  10,   1,    0,           0, 
                    1,   1,    0.1,  0,           0, 
                    0,   0,    0,    pow(10,-8),  0, 
                    0,   0,    0,    0,           pow(10,-8) ;

            EKF.Set_P(P_in) ;

            Eigen::MatrixXd Q_in(5, 5) ;
            Q_in << 0.01,  0.0,   0.0,    0.0,  0.0, 
                    0.0,   0.01,  0.0,    0.0,  0.0, 
                    0.0,   0.0,   0.0001, 0.0,  0.0, 
                    0.0,   0.0,   0.0,    0.01, 0.0, 
                    0.0,   0.0,   0.0,    0.0,  0.01 ;
            EKF.SetQ(Q_in) ;

            //R is provided by Sensor producer, in their datasheet
            Eigen::MatrixXd R_in(3, 3) ;
            R_in << 0.0001, 0,      0, 
                    0,      0.0001, 0,
                    0,      0,      0.0001 ;
            EKF.SetR(R_in) ;
        }

        //state forward eqution
        EKF.PredictX(delta_time) ;
        EKF.Set_A(delta_time) ; //first update x, the calculate A
        EKF.PredictP() ;        // state convarince matrix
        EKF.SetH() ;            // measurement convarince matrix

        // measurement value
        Eigen::VectorXd z(measure_x[0]) ;
        EKF.MeasurementUpdate(z) ;

        //result
        Eigen::VectorXd x_out = EKF.GetX() ;
        std::cout << "original output x : " << x_noise[0](0) <<
                                    " y: " << x_noise[0](1) << std::endl ;
        std::cout << "ideal   output x : " << x_stand[0](0) <<
                                    " y: " << x_stand[0](1) << std::endl ;
        std::cout << "kalman output x : " << x_out(0) <<
                                    " y: " << x_out(1) << std::endl << std::endl;

        //////////for painting//////////
        vis_x_ekf.push_back(x_out(0)) ;
        vis_y_ekf.push_back(x_out(1)) ;
        vis_x_nse.push_back(x_noise[0](0)) ;
        vis_y_nse.push_back(x_noise[0](1)) ;
        vis_x_std.push_back(x_stand[0](0)) ;
        vis_y_std.push_back(x_stand[0](1)) ;

        x_noise.erase(x_noise.begin()) ;
        x_stand.erase(x_stand.begin()) ;
        measure_x.erase(measure_x.begin()) ;
    }

        plt::figure_size(1200, 780);
        plt::plot(vis_x_ekf, vis_y_ekf,"r");
        plt::plot(vis_x_nse, vis_y_nse,"g");
        plt::plot(vis_x_std, vis_y_std,"b");
        plt::title("ekf-red, noise-green, stand-blue");
        plt::save("./basic.png");
    //     //////////for painting//////////
    return 0;
}