#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <variant>

#include "core/proto/factor.pb.h"
#include "core/proto/geometry.pb.h"
#include "core/types/pose.hpp"
#include "core/types/imu.hpp"

namespace core {
namespace types {

struct Factor {
    uint64_t id;
    proto::FactorType type;
    std::vector<uint64_t> connected_nodes;

    // Factor measurement (pose or IMU preintegrated measurement)
    std::variant<types::Pose, types::Pose, types::ImuPreintegratedMeasurement> measurement;

    // Information matrix (inverse covariance) - 6x6 for poses, 9x9 for IMU
    Eigen::MatrixXd information;
    double timestamp;
    std::string description;

    proto::Factor toProto() const {
        proto::Factor factor_proto;
        factor_proto.set_id(id);
        factor_proto.set_type(type);

        for (const auto& node_id : connected_nodes) {
            factor_proto.add_connected_nodes(node_id);
        }

        switch (type) {
            case proto::FactorType::PRIOR: {
                *factor_proto.mutable_absolute_pose() = std::get<0>(measurement).toProto();
                break;
            }
            case proto::FactorType::ODOMETRY:
            case proto::FactorType::LOOP_CLOSURE: {
                *factor_proto.mutable_relative_pose() = std::get<1>(measurement).toProto();
                break;
            }
            case proto::FactorType::IMU_PREINTEGRATED: {
                const auto& imu_meas = std::get<2>(measurement);
                auto* imu_proto = factor_proto.mutable_imu_preintegrated();

                // Convert preintegrated measurements
                auto* pos = imu_proto->mutable_preintegrated_position();
                pos->set_x(imu_meas.preintegrated_position.x());
                pos->set_y(imu_meas.preintegrated_position.y());
                pos->set_z(imu_meas.preintegrated_position.z());

                auto* vel = imu_proto->mutable_preintegrated_velocity();
                vel->set_x(imu_meas.preintegrated_velocity.x());
                vel->set_y(imu_meas.preintegrated_velocity.y());
                vel->set_z(imu_meas.preintegrated_velocity.z());

                auto* rot = imu_proto->mutable_preintegrated_rotation();
                rot->set_w(imu_meas.preintegrated_rotation.w());
                rot->set_x(imu_meas.preintegrated_rotation.x());
                rot->set_y(imu_meas.preintegrated_rotation.y());
                rot->set_z(imu_meas.preintegrated_rotation.z());

                imu_proto->set_delta_time(imu_meas.delta_time);

                // Convert biases
                auto* acc_bias = imu_proto->mutable_accelerometer_bias();
                acc_bias->set_x(imu_meas.accelerometer_bias.x());
                acc_bias->set_y(imu_meas.accelerometer_bias.y());
                acc_bias->set_z(imu_meas.accelerometer_bias.z());

                auto* gyro_bias = imu_proto->mutable_gyroscope_bias();
                gyro_bias->set_x(imu_meas.gyroscope_bias.x());
                gyro_bias->set_y(imu_meas.gyroscope_bias.y());
                gyro_bias->set_z(imu_meas.gyroscope_bias.z());

                // Convert covariance matrix
                for (int i = 0; i < 81; ++i) {  // 9x9 matrix
                    imu_proto->add_covariance_matrix(imu_meas.covariance_matrix.data()[i]);
                }
                break;
            }
            default:
                break;
        }

        // Convert information matrix
        for (int i = 0; i < information.size(); ++i) {
            factor_proto.add_information_matrix(information.data()[i]);
        }

        factor_proto.set_timestamp(timestamp);
        factor_proto.set_description(description);
        return factor_proto;
    }

    static Factor fromProto(const proto::Factor& factor_proto) {
        Factor factor;
        factor.id = factor_proto.id();
        factor.type = factor_proto.type();

        factor.connected_nodes.reserve(factor_proto.connected_nodes_size());
        for (const auto& node_id : factor_proto.connected_nodes()) {
            factor.connected_nodes.push_back(node_id);
        }

        // Handle different measurement types
        switch (factor_proto.type()) {
            case proto::FactorType::PRIOR: {
                factor.measurement.emplace<0>(types::Pose::fromProto(factor_proto.absolute_pose()));
                factor.information = Eigen::Matrix<double, 6, 6>::Map(
                    factor_proto.information_matrix().data());
                break;
            }
            case proto::FactorType::ODOMETRY:
            case proto::FactorType::LOOP_CLOSURE: {
                factor.measurement.emplace<1>(types::Pose::fromProto(factor_proto.relative_pose()));
                factor.information = Eigen::Matrix<double, 6, 6>::Map(
                    factor_proto.information_matrix().data());
                break;
            }
            case proto::FactorType::IMU_PREINTEGRATED: {
                types::ImuPreintegratedMeasurement imu_meas;
                const auto& imu_proto = factor_proto.imu_preintegrated();

                // Convert preintegrated measurements
                imu_meas.preintegrated_position = Eigen::Vector3d(
                    imu_proto.preintegrated_position().x(),
                    imu_proto.preintegrated_position().y(),
                    imu_proto.preintegrated_position().z());

                imu_meas.preintegrated_velocity = Eigen::Vector3d(
                    imu_proto.preintegrated_velocity().x(),
                    imu_proto.preintegrated_velocity().y(),
                    imu_proto.preintegrated_velocity().z());

                imu_meas.preintegrated_rotation = Eigen::Quaterniond(
                    imu_proto.preintegrated_rotation().w(),
                    imu_proto.preintegrated_rotation().x(),
                    imu_proto.preintegrated_rotation().y(),
                    imu_proto.preintegrated_rotation().z());

                imu_meas.delta_time = imu_proto.delta_time();

                // Convert biases
                imu_meas.accelerometer_bias = Eigen::Vector3d(
                    imu_proto.accelerometer_bias().x(),
                    imu_proto.accelerometer_bias().y(),
                    imu_proto.accelerometer_bias().z());

                imu_meas.gyroscope_bias = Eigen::Vector3d(
                    imu_proto.gyroscope_bias().x(),
                    imu_proto.gyroscope_bias().y(),
                    imu_proto.gyroscope_bias().z());

                // Convert covariance matrix
                imu_meas.covariance_matrix = Eigen::Matrix<double, 9, 9>::Map(
                    imu_proto.covariance_matrix().data());

                factor.measurement.emplace<2>(imu_meas);
                factor.information = Eigen::Matrix<double, 9, 9>::Map(
                    factor_proto.information_matrix().data());
                break;
            }
            default:
                factor.measurement.emplace<0>(types::Pose{});
                factor.information = Eigen::Matrix<double, 6, 6>::Identity();
                break;
        }

        factor.timestamp = factor_proto.timestamp();
        factor.description = factor_proto.description();
        return factor;
    }
};

}  // namespace types
}  // namespace core
