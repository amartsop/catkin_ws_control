#pragma once
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

class LowPassFilterEigen
{
  public:  

    /**
     * @brief Low-pass filter constructor.
     * @param f0: cutoff frequency (Hz)
     * @param fs: sample frequency (Hz)
     * @param adaptive: boolean flag, if set to 1, the code will automatically
     * set the sample frequency based on the time history.
    */ 
    LowPassFilterEigen(int order, double f0, double fs, int rows, int cols,
        bool adaptive);

    // Get filter 
    Eigen::MatrixXd filt(const Eigen::MatrixXd& xn);

  private:

    // Filter order 
    int m_order;

    // Vector of a coeffs
    std::vector<double> m_a_coeffs; 

    // Vector of b coeffs
    std::vector<double> m_b_coeffs;

    // Cutoff frequency (rad/sec)
    double m_omega0;

    // Sampling period
    double m_dt;

    // Adaptive flag
    bool m_adaptive_flag;    

    // Previous time
    double m_tn1 = 0;

private:

    // Raw input values
    std::vector<Eigen::MatrixXd> m_x_vec;

    // Raw filtered values
    std::vector<Eigen::MatrixXd> m_y_vec;

    // Matrix/Vector rows and cols
    int m_rows, m_cols;

    // Set coefficinents 
    void set_coeffs(void);
};

