#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <string>

namespace core {
namespace types {

struct ImuData {
    // Linear acceleration (m/s^2)
    Eigen::Vector3d linear_acceleration;
    Eigen::Matrix3d linear_acceleration_covariance;

    // Angular velocity (rad/s)
    Eigen::Vector3d angular_velocity;
    Eigen::Matrix3d angular_velocity_covariance;

    // Orientation (if available from IMU)
    Eigen::Quaterniond orientation;
    Eigen::Matrix3d orientation_covariance;

    // Metadata
    double timestamp = 0.0;
    std::string frame_id;
    uint64_t sequence = 0;

    ImuData() {
        linear_acceleration = Eigen::Vector3d::Zero();
        linear_acceleration_covariance = Eigen::Matrix3d::Identity();

        angular_velocity = Eigen::Vector3d::Zero();
        angular_velocity_covariance = Eigen::Matrix3d::Identity();

        orientation = Eigen::Quaterniond::Identity();
        orientation_covariance = Eigen::Matrix3d::Identity();
    }
};

struct ImuPreintegratedMeasurement {
    // Preintegrated measurements between two poses
    Eigen::Vector3d preintegrated_position;
    Eigen::Vector3d preintegrated_velocity;
    Eigen::Quaterniond preintegrated_rotation;

    // Covariance matrix (9x9 for position, velocity, rotation)
    Eigen::Matrix<double, 9, 9> covariance_matrix;

    // Time interval
    double delta_time = 0.0;

    // Bias estimates
    Eigen::Vector3d accelerometer_bias;
    Eigen::Vector3d gyroscope_bias;

    ImuPreintegratedMeasurement() {
        preintegrated_position = Eigen::Vector3d::Zero();
        preintegrated_velocity = Eigen::Vector3d::Zero();
        preintegrated_rotation = Eigen::Quaterniond::Identity();
        covariance_matrix = Eigen::Matrix<double, 9, 9>::Identity();
        accelerometer_bias = Eigen::Vector3d::Zero();
        gyroscope_bias = Eigen::Vector3d::Zero();
    }
};

}  // namespace types
}  // namespace core