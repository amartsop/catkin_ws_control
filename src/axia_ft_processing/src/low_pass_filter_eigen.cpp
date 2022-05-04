#include "../include/low_pass_filter_eigen.h"

LowPassFilterEigen::LowPassFilterEigen(int order, double f0, double fs,
    int rows, int cols, bool adaptive)
{
    // Set filter order
    m_order = order;    

    // Matrix/Vector rows and cols
    m_rows = rows; m_cols = cols;

    // Initialize coeffs
    m_a_coeffs.resize(m_order);
    m_b_coeffs.resize(m_order+1);

    // Set cuttof frequency (rad/sec)
    m_omega0 = 2.0 * M_PI * f0;

    // Set sampling period
    m_dt = 1.0 / fs;

    // Set adaptive flag
    m_adaptive_flag = adaptive;

    // Set previous time
    m_tn1 = - m_dt;

    // Resize input and filtered values 
    m_x_vec.resize(m_order + 1);
    m_y_vec.resize(m_order + 1);

    // Initialize vectors
    for (int k = 0; k < m_order + 1; k++)
    {
        m_x_vec[k] = Eigen::MatrixXd::Zero(rows, cols);
        m_y_vec[k] = Eigen::MatrixXd::Zero(rows, cols);
    }

    // Set coefficients
    set_coeffs();
    
}

void LowPassFilterEigen::set_coeffs(void)
{
    // if(adapt){
    // float t = micros()/1.0e6;
    // dt = t - tn1;
    // tn1 = t;
    // }
      
    // Calculate alpha
    double alpha = m_omega0 * m_dt;
    
    // Calculate coefficiends for a order 1 filter
    if(m_order == 1)
    {
        m_a_coeffs[0] = - (alpha - 2.0) / (alpha + 2.0);
        m_b_coeffs[0] = alpha / (alpha + 2.0);
        m_b_coeffs[1] = alpha / (alpha + 2.0);        
    }
    
    // Calculate coefficiends for a order 1 filter
    if(m_order == 2)
    {
        double c1 = 2.0 * std::sqrt(2.0) / alpha;
        double c2 = 4.0 / (alpha * alpha);
        double denom = 1.0 + c1 + c2;

        m_b_coeffs[0] = 1.0 / denom;
        m_b_coeffs[1] = 2.0 / denom;
        m_b_coeffs[2] = m_b_coeffs.at(0);
        m_a_coeffs[0] = - (2.0 - 2.0 * c2) / denom;
        m_a_coeffs[1] = - (1.0 - c1 + c2) / (1.0 + c1 + c2);      
    }
}


Eigen::MatrixXd LowPassFilterEigen::filt(const Eigen::MatrixXd& xn)
{
    if(m_adaptive_flag)
    {
    // setCoef(); // Update coefficients if necessary      
    }

    // Initialize input and filtered values
    m_y_vec[0] = Eigen::MatrixXd::Zero(m_rows, m_cols);
    m_x_vec[0] = xn;

    // Compute the filtered values
    for(int k = 0; k < m_order; k++)
    {
        m_y_vec[0] += m_a_coeffs[k] * m_y_vec[k+1] + m_b_coeffs[k] * m_x_vec[k];
    }

    m_y_vec[0] += m_b_coeffs[m_order] * m_x_vec[m_order];

    // Save the historical values
    for(int k = m_order; k > 0; k--)
    {
        m_y_vec[k] = m_y_vec[k-1];
        m_x_vec[k] = m_x_vec[k-1];
    }
  
    // Return the filtered value    
    return m_y_vec[0];
}