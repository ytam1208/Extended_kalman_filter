#ifndef __KALMAN_FILTER_HPP__
#define __KALMAN_FILTER_HPP__

#include <cstdint>
#include <cmath>

#include <Eigen/Dense>

namespace ekf {
using float32_t = float;

template<size_t ROW, size_t COL>
using Matrix = Eigen::Matrix<float32_t, ROW, COL>;

template<size_t ROW>
using Vector = Eigen::Matrix<float32_t, ROW, 1>;

///
/// @brief DIM_X: dimension of state vector
/// @brief DIM_Z: dimension of measurement vector
template<size_t DIM_X, size_t DIM_Z> 
class ExtendedKalmanFilter {
private:
    /// @brief estimated state vector
    Vector<DIM_X> vec_x_;
    /// @brief state covariance matrix
    Matrix<DIM_X, DIM_X> mat_p_;  
    /// @brief motion noise covariance matrix
    Matrix<DIM_X, DIM_X> q_;

public:
    ExtendedKalmanFilter() {
        vec_x_.setZero();
        mat_p_.setIdentity();
        q_.setIdentity();
        q_ *= 0.05;      // Adjust this value based on your system
    }
    ~ExtendedKalmanFilter(){}

    Vector<DIM_X>& vecX(void) { return vec_x_; }
    const Vector<DIM_X>& vecX(void) const { return vec_x_; }
    Matrix<DIM_X, DIM_X>& matP(void) { return mat_p_; }
    const Matrix<DIM_X, DIM_X>& matP(void) const { return mat_p_; }

    /// @brief velocity motion model (2-wheel robot)
    /// @param velocity linear_vel, angular_vel
    /// @param dt 
    void MotionModel(const Vector<DIM_X>& velocity, const float32_t dt) {
        float32_t x_vel = velocity(0);  // Linear velocity 
        float32_t yaw_vel = velocity(2); // Angular velocity (yaw rate)

        float32_t x = vec_x_(0);
        float32_t y = vec_x_(1);
        float32_t yaw = vec_x_(2);

        // Update the state using the velocity and time step
        float32_t new_x = x + (x_vel * std::cos(yaw)) * dt;
        float32_t new_y = y + (x_vel * std::sin(yaw)) * dt;
        float32_t new_yaw = yaw + yaw_vel * dt;

        // Optionally, normalize yaw to keep it within [-pi, pi]
        new_yaw = std::atan2(std::sin(new_yaw), std::cos(new_yaw));

        vec_x_(0) = new_x;
        vec_x_(1) = new_y;
        vec_x_(2) = new_yaw;
    }

    /// @brief observation model
    /// @return measurement noise covariance H matrix
    Matrix<DIM_Z, DIM_X> ObservationModel(void) const {
        /*
            Why observation model is identity matrix? 
            because the measurement is the same as the state vector (x, y, yaw) = (x, y, yaw)
            If the measurement is different from the state vector, you need to define the observation model matrix H
        */
        Matrix<DIM_Z, DIM_X> h;
        h.setIdentity();
        return h;
    }

    /// @brief Prediction step
    /// @param velocity linear_vel, angular_vel
    /// @param dt 
    void Predict(const Vector<DIM_X>& velocity, const float32_t dt) {
        MotionModel(velocity, dt);

        // State transition matrix (Jacobian of the motion model)
        // non-holonomic motion model
        // v * sin(yaw) * dt : x coordinate is changing based on the direction
        // v * cos(yaw) * dt : y coordinate is changing based on the direction
        Matrix<DIM_X, DIM_X> F;
        F.setIdentity();
        F(0, 2) = -velocity(0) * std::sin(vec_x_(2)) * dt;
        F(1, 2) = velocity(0) * std::cos(vec_x_(2)) * dt;

        mat_p_ = F * mat_p_ * F.transpose() + q_;
    }

    /// @brief correct state of with a linear measurement model. 
    /// @param vecZ: measurement of model
    /// @param mat_r: measurement noise covariance matrix of model
    /// @param mat_h: measurement matrix of model
    void Correct(const Vector<DIM_Z>& vec_z, 
                 const Matrix<DIM_Z, DIM_Z>& mat_r, 
                 const Matrix<DIM_Z, DIM_X>& mat_h) {
        // Identity matrix
        const Matrix<DIM_X, DIM_X> mat_i{ 
            Matrix<DIM_X, DIM_X>::Identity() };             
    
        // Innovation covariance
        const Matrix<DIM_Z, DIM_Z> mat_s_t{ 
            mat_h * mat_p_ * mat_h.transpose() + mat_r }; 
        
        // Kalman gain
        const Matrix<DIM_X, DIM_Z> mat_k_t{ 
            mat_p_ * mat_h.transpose() * mat_s_t.inverse() }; 

        // Update all state variables (x, y, yaw)
        vec_x_ = vec_x_ + mat_k_t * (vec_z - mat_h * vec_x_);
        mat_p_ = (mat_i - mat_k_t * mat_h) * mat_p_;
    }

    /// @brief correct the state with the IMU yaw data
    /// @param imu_yaw: yaw angle from IMU
    /// @param yaw_variance: variance of yaw angle
    void Correct(const float32_t imu_yaw, const float32_t yaw_variance) {
        // Measurement vector (only yaw)
        Vector<DIM_Z> vec_z;
        vec_z(0) = imu_yaw;

        // Measurement noise covariance matrix (only yaw)
        Matrix<DIM_Z, DIM_Z> mat_r;
        mat_r(0, 0) = yaw_variance;

        // Measurement matrix (only yaw)
        Matrix<DIM_Z, DIM_X> mat_h;
        mat_h.setZero();
        mat_h(0, 2) = 1.0; // Only yaw is measured
        // Correct the state with the IMU yaw data

        // Identity matrix
        const Matrix<DIM_X, DIM_X> mat_i{ 
            Matrix<DIM_X, DIM_X>::Identity() };         

        // Innovation covariance
        const Matrix<DIM_Z, DIM_Z> mat_s_t{ 
            mat_h * mat_p_ * mat_h.transpose() + mat_r };

        // Kalman gain
        const Matrix<DIM_X, DIM_Z> mat_k_t{ 
            mat_p_ * mat_h.transpose() * mat_s_t.inverse() };     
            
        // Measurement - Predicted value
        Eigen::Matrix<float32_t, DIM_Z, DIM_Z> y;
        y(0, 0) = vec_z(0) - vec_x_(2); 
        vec_x_ = vec_x_ + mat_k_t * y;
        mat_p_ = (mat_i - mat_k_t * mat_h) * mat_p_;
    }
};
}

#endif