#include "attitude/filter.hpp"

#include <Eigen/Cholesky>
#include <cmath>
#include <stdexcept>
#include <unsupported/Eigen/MatrixFunctions>

namespace attitude {
namespace {
void validate_noise(Noise noise) {
    if (!(std::isfinite(noise.gyro) && noise.gyro >= 0 && std::isfinite(noise.bias_walk) &&
          noise.bias_walk >= 0)) {
        throw std::invalid_argument("noise densities must be finite and nonnegative");
    }
}
void validate_time(double dt) {
    if (!(std::isfinite(dt) && dt > 0 && dt <= 10)) {
        throw std::invalid_argument("gyro interval must be in (0, 10] seconds");
    }
}
void check_covariance(const Mat6 &p) {
    if (!p.allFinite() || Eigen::LLT<Mat6>(p).info() != Eigen::Success) {
        throw std::runtime_error("filter covariance lost positive definiteness");
    }
}
} // namespace

DiscreteModel discretize(const Vec3 &rate, Noise noise, double dt) {
    validate_noise(noise);
    validate_time(dt);
    if (!rate.allFinite() || rate.norm() > 100) {
        throw std::invalid_argument("corrected rate must be finite and at most 100 rad/s");
    }
    Mat6 f = Mat6::Zero();
    f.topLeftCorner<3, 3>() = -skew(rate);
    f.topRightCorner<3, 3>() = -Mat3::Identity();
    Mat6 spectral = Mat6::Zero();
    spectral.topLeftCorner<3, 3>() = Mat3::Identity() * noise.gyro * noise.gyro;
    spectral.bottomRightCorner<3, 3>() = Mat3::Identity() * noise.bias_walk * noise.bias_walk;

    // Van Loan block exponential integrates process noise over the interval.
    // Multiplying the upper-right block by Phi^T is essential: that block alone
    // is not Q_d and is generally neither symmetric nor a valid covariance.
    Eigen::Matrix<double, 12, 12> block = Eigen::Matrix<double, 12, 12>::Zero();
    block.topLeftCorner<6, 6>() = f;
    block.topRightCorner<6, 6>() = spectral;
    block.bottomRightCorner<6, 6>() = -f.transpose();
    const Eigen::Matrix<double, 12, 12> exponential = (block * dt).exp();
    const Mat6 transition = exponential.topLeftCorner<6, 6>();
    const Mat6 raw_q = exponential.topRightCorner<6, 6>() * transition.transpose();
    if (!transition.allFinite() || !raw_q.allFinite()) {
        throw std::runtime_error("discrete covariance model overflow");
    }
    return {transition, (raw_q + raw_q.transpose()) * 0.5};
}

Filter::Filter(Quaternion initial, Config config) : config_(config) {
    validate_noise(config.noise);
    (void)rotation_log(initial);
    for (double value : {config.angle_std, config.bias_std, config.gate_squared}) {
        if (!(std::isfinite(value) && value > 0))
            throw std::invalid_argument("invalid filter configuration");
    }
    estimate_.orientation = initial.normalized();
    estimate_.covariance = Mat6::Zero();
    estimate_.covariance.topLeftCorner<3, 3>() =
        Mat3::Identity() * config.angle_std * config.angle_std;
    estimate_.covariance.bottomRightCorner<3, 3>() =
        Mat3::Identity() * config.bias_std * config.bias_std;
    check_covariance(estimate_.covariance);
}

void Filter::propagate(const Vec3 &measured_rate, double dt) {
    const Vec3 rate = measured_rate - estimate_.bias;
    const auto model = discretize(rate, config_.noise, dt);
    const Mat6 raw = model.transition * estimate_.covariance * model.transition.transpose() +
                     model.process_covariance;
    const Mat6 covariance = (raw + raw.transpose()) * 0.5;
    check_covariance(covariance);
    // Build the complete candidate before mutating state, including on errors.
    const Quaternion orientation = (estimate_.orientation * rotation_exp(rate * dt)).normalized();
    estimate_.orientation = orientation;
    estimate_.covariance = covariance;
}

Observation Filter::observe(Quaternion tracker, const Mat3 &measurement_covariance) {
    (void)rotation_log(tracker);
    const Mat3 &r = measurement_covariance;
    if (!r.allFinite() || !r.isApprox(r.transpose(), 1e-12) ||
        Eigen::LLT<Mat3>(r).info() != Eigen::Success) {
        throw std::invalid_argument("tracker covariance must be symmetric positive definite");
    }
    const Vec3 residual = rotation_log(estimate_.orientation.conjugate() * tracker.normalized());
    const Mat3 innovation_covariance = estimate_.covariance.topLeftCorner<3, 3>() + r;
    const Eigen::LDLT<Mat3> solve(innovation_covariance);
    if (solve.info() != Eigen::Success || !solve.isPositive()) {
        throw std::runtime_error("innovation covariance cannot be solved");
    }
    const double distance = residual.dot(solve.solve(residual));
    if (!std::isfinite(distance))
        throw std::runtime_error("nonfinite innovation distance");
    if (distance > config_.gate_squared)
        return {false, distance, residual};

    // Solve S*K^T = (P*H^T)^T instead of forming a matrix inverse. H = [I, 0].
    const Eigen::Matrix<double, 6, 3> gain =
        solve.solve(estimate_.covariance.leftCols<3>().transpose()).transpose();
    const Eigen::Matrix<double, 6, 1> correction = gain * residual;
    Mat6 identity_minus_kh = Mat6::Identity();
    identity_minus_kh.leftCols<3>() -= gain;
    const Mat6 posterior =
        identity_minus_kh * estimate_.covariance * identity_minus_kh.transpose() +
        gain * r * gain.transpose();

    // The attitude correction moves the origin of the local error coordinates.
    // Transport covariance into that new tangent frame, including angle/bias
    // cross-covariances. Normalizing q alone would not perform this reset.
    Mat6 reset = Mat6::Identity();
    reset.topLeftCorner<3, 3>() = right_jacobian(correction.head<3>());
    const Mat6 transported = reset * posterior * reset.transpose();
    const Mat6 covariance = (transported + transported.transpose()) * 0.5;
    check_covariance(covariance);
    const Quaternion orientation =
        (estimate_.orientation * rotation_exp(correction.head<3>())).normalized();
    const Vec3 bias = estimate_.bias + correction.tail<3>();
    if (!bias.allFinite())
        throw std::runtime_error("nonfinite estimated bias");
    estimate_ = {orientation, bias, covariance};
    return {true, distance, residual};
}
} // namespace attitude
