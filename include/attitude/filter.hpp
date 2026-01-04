#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace attitude {
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Quaternion = Eigen::Quaterniond;

// Hamilton quaternions; q * v rotates a body-frame vector into the inertial frame.
// Body angular rate advances attitude as q_next = q * Exp(rate * dt).
// Eigen's coefficient storage is x,y,z,w; its constructor takes w,x,y,z.
Mat3 skew(const Vec3 &vector);
Quaternion rotation_exp(const Vec3 &rotation_vector);
Vec3 rotation_log(Quaternion rotation); // shortest rotation, magnitude <= pi
Mat3 right_jacobian(const Vec3 &rotation_vector);

struct Noise {
    double gyro{5e-4};      // rad / sqrt(s), rate white-noise density
    double bias_walk{2e-6}; // (rad/s) / sqrt(s)
};
struct DiscreteModel {
    Mat6 transition;
    Mat6 process_covariance;
};

// Right attitude error: q_true = q_estimate * Exp(delta_angle).
// Error state is [delta_angle (rad), true_bias - estimated_bias (rad/s)].
// Integrates this linearized continuous error model for constant angular rate.
DiscreteModel discretize(const Vec3 &corrected_rate, Noise noise, double dt);

struct Estimate {
    Quaternion orientation{Quaternion::Identity()};
    Vec3 bias{Vec3::Zero()};
    Mat6 covariance{Mat6::Identity()};
};
struct Config {
    Noise noise;
    double angle_std{0.15};
    double bias_std{0.005};
    double gate_squared{16.27}; // threshold on residual^T S^-1 residual
};
struct Observation {
    bool accepted;
    double innovation_squared;
    Vec3 residual;
};

class Filter {
  public:
    explicit Filter(Quaternion initial, Config config = {});
    // measured_rate is the body-frame gyro rate in rad/s, averaged over dt.
    // dt must be in (0, 10] seconds. Feed each gyro interval during tracker loss.
    void propagate(const Vec3 &measured_rate, double dt);
    // Tracker covariance is a 3x3 rotation-vector covariance in body-frame rad^2,
    // not a covariance over four quaternion coefficients. A rejected outlier
    // leaves the orientation, bias, and covariance unchanged.
    Observation observe(Quaternion tracker_orientation, const Mat3 &tracker_covariance);
    [[nodiscard]] const Estimate &estimate() const {
        return estimate_;
    }

  private:
    Config config_;
    Estimate estimate_;
};
} // namespace attitude
