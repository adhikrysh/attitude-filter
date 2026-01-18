#include "attitude/filter.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>

namespace {
using namespace attitude;
void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
template <class F> void rejects(F f) {
    bool rejected = false;
    try {
        f();
    } catch (const std::exception &) {
        rejected = true;
    }
    require(rejected, "invalid input accepted");
}
Quaternion independent_rotation(const Vec3 &vector) {
    if (vector.norm() == 0)
        return Quaternion::Identity();
    return Quaternion(Eigen::AngleAxisd(vector.norm(), vector.normalized()));
}

void conventions() {
    const auto rotation = rotation_exp({0, 0, std::numbers::pi / 2});
    require((rotation * Vec3{1, 0, 0} - Vec3{0, 1, 0}).norm() < 1e-14, "wrong handedness");
    for (Vec3 vector : {Vec3{1e-12, -2e-12, 3e-12}, Vec3{0.2, -0.5, 0.7}, Vec3{2, 0.3, -0.5}}) {
        const auto q = rotation_exp(vector);
        require((q.toRotationMatrix() - independent_rotation(vector).toRotationMatrix()).norm() <
                    1e-14,
                "exponential differs from angle-axis");
        require((rotation_log(q) - vector).norm() < 1e-14, "log/exp mismatch");
        auto negated = q;
        negated.coeffs() *= -1;
        require((rotation_log(q) - rotation_log(negated)).norm() < 1e-14,
                "quaternion sign affects rotation error");
    }
}

void jacobians() {
    constexpr double epsilon = 1e-6;
    for (Vec3 vector : {Vec3{1e-7, 2e-7, 0}, Vec3{0.2, -0.3, 0.6}}) {
        Mat3 finite_difference;
        for (int axis = 0; axis < 3; ++axis) {
            Vec3 delta = Vec3::Zero();
            delta[axis] = epsilon;
            const auto origin = independent_rotation(vector).conjugate();
            finite_difference.col(axis) =
                (rotation_log(origin * independent_rotation(vector + delta)) -
                 rotation_log(origin * independent_rotation(vector - delta))) /
                (2 * epsilon);
        }
        require((finite_difference - right_jacobian(vector)).norm() < 2e-9,
                "reset Jacobian differs from finite differences");
    }
    const Vec3 rate{0.13, -0.21, 0.07};
    constexpr double dt = 0.3;
    const Quaternion nominal = independent_rotation(rate * dt);
    Mat6 numeric = Mat6::Zero();
    for (int axis = 0; axis < 6; ++axis) {
        auto advance_error = [&](double sign) {
            Vec3 angle = Vec3::Zero(), bias = Vec3::Zero();
            if (axis < 3)
                angle[axis] = sign * epsilon;
            else
                bias[axis - 3] = sign * epsilon;
            const Quaternion actual =
                independent_rotation(angle) * independent_rotation((rate - bias) * dt);
            Eigen::Matrix<double, 6, 1> error;
            error << rotation_log(nominal.conjugate() * actual), bias;
            return error;
        };
        numeric.col(axis) = (advance_error(1) - advance_error(-1)) / (2 * epsilon);
    }
    require((numeric - discretize(rate, {}, dt).transition).norm() < 2e-9,
            "transition Jacobian differs from nonlinear motion");
}

void process_noise() {
    constexpr double dt = 0.4;
    const Noise noise{0.004, 0.003};
    const auto discrete = discretize(Vec3::Zero(), noise, dt);
    Mat6 expected = Mat6::Zero();
    expected.topLeftCorner<3, 3>() =
        Mat3::Identity() *
        (noise.gyro * noise.gyro * dt + noise.bias_walk * noise.bias_walk * dt * dt * dt / 3);
    expected.topRightCorner<3, 3>() =
        Mat3::Identity() * (-noise.bias_walk * noise.bias_walk * dt * dt / 2);
    expected.bottomLeftCorner<3, 3>() = expected.topRightCorner<3, 3>();
    expected.bottomRightCorner<3, 3>() = Mat3::Identity() * noise.bias_walk * noise.bias_walk * dt;
    require((discrete.process_covariance - expected).norm() < 1e-17,
            "process covariance differs from zero-rate closed form");
    for (Vec3 rate : {Vec3{0, 0, 0}, Vec3{0.2, -0.7, 0.4}, Vec3{12, 3, -7}}) {
        const auto model = discretize(rate, noise, 0.2);
        require(model.process_covariance.isApprox(model.process_covariance.transpose(), 1e-13),
                "process covariance asymmetric");
        require(
            Eigen::SelfAdjointEigenSolver<Mat6>(model.process_covariance).eigenvalues().minCoeff() >
                0,
            "process covariance is not positive");
        const auto half = discretize(rate, noise, 0.1);
        const Mat6 composed =
            half.transition * half.process_covariance * half.transition.transpose() +
            half.process_covariance;
        require((composed - model.process_covariance).norm() < 1e-17,
                "noise discretization does not compose");
    }
}

void gating_and_errors() {
    Config config;
    config.angle_std = 0.001;
    Filter filter(Quaternion::Identity(), config);
    const auto before = filter.estimate();
    const auto outlier = filter.observe(rotation_exp({1, 0, 0}), Mat3::Identity() * 1e-6);
    require(!outlier.accepted, "large tracker error was not gated");
    require(filter.estimate().orientation.coeffs() == before.orientation.coeffs(),
            "gated observation changed orientation");
    require(filter.estimate().covariance == before.covariance,
            "gated observation changed covariance");
    rejects([&] { filter.propagate(Vec3::Zero(), 0); });
    rejects(
        [&] { filter.propagate(Vec3::Constant(std::numeric_limits<double>::quiet_NaN()), 0.1); });
    rejects([&] { (void)filter.observe(Quaternion::Identity(), -Mat3::Identity()); });
    rejects([] { (void)Filter(Quaternion(0, 0, 0, 0)); });
    require(filter.estimate().covariance == before.covariance, "failed call mutated covariance");
}

void monte_carlo() {
    constexpr int trials = 24;
    constexpr double dt = 0.2, tracker_std = 0.002;
    const Config config;
    double mean_nees = 0, angle_squared = 0, bias_squared = 0;
    for (int seed = 0; seed < trials; ++seed) {
        std::mt19937_64 engine(static_cast<unsigned>(seed) + 1234);
        std::normal_distribution<double> normal;
        auto normal3 = [&]() { return Vec3{normal(engine), normal(engine), normal(engine)}; };
        Quaternion truth = Quaternion::Identity();
        Vec3 bias{0.001, -0.0015, 0.0008};
        Filter filter(independent_rotation({0.06, -0.04, 0.03}), config);
        bool rejected_injected_outlier = false;
        for (int step = 1; step <= 1000; ++step) {
            const double t = step * dt;
            const Vec3 rate{0.01 * std::sin(t / 30), 0.013 * std::cos(t / 20), 0.02};
            truth = (truth * independent_rotation(rate * dt)).normalized();
            const Vec3 measurement = rate + bias + normal3() * (config.noise.gyro / std::sqrt(dt));
            bias += normal3() * (config.noise.bias_walk * std::sqrt(dt));
            filter.propagate(measurement, dt);
            if (step % 5 == 0 && !(t >= 60 && t < 100)) {
                auto tracker = truth * independent_rotation(normal3() * tracker_std);
                if (step == 600)
                    tracker = tracker * independent_rotation({0.6, 0, 0});
                if (step % 10 == 0)
                    tracker.coeffs() *= -1;
                const auto observation =
                    filter.observe(tracker, Mat3::Identity() * tracker_std * tracker_std);
                if (step == 600)
                    rejected_injected_outlier = !observation.accepted;
            }
            const auto &covariance = filter.estimate().covariance;
            require(covariance.isApprox(covariance.transpose(), 1e-12), "covariance symmetry lost");
            require(Eigen::LLT<Mat6>(covariance).info() == Eigen::Success,
                    "covariance positive definiteness lost");
            require(std::abs(filter.estimate().orientation.norm() - 1) < 1e-14,
                    "quaternion norm drift");
        }
        require(rejected_injected_outlier, "injected tracker failure accepted");
        const auto &estimate = filter.estimate();
        Eigen::Matrix<double, 6, 1> error;
        error << rotation_log(estimate.orientation.conjugate() * truth), bias - estimate.bias;
        angle_squared += error.head<3>().squaredNorm();
        bias_squared += error.tail<3>().squaredNorm();
        mean_nees += error.dot(estimate.covariance.ldlt().solve(error));
    }
    mean_nees /= trials;
    const double angle_rms = std::sqrt(angle_squared / trials);
    const double bias_rms = std::sqrt(bias_squared / trials);
    std::cout << "ensemble angle_rms_rad=" << angle_rms << " bias_rms_rad_s=" << bias_rms
              << " mean_normalized_squared_error=" << mean_nees << '\n';
    require(angle_rms < 0.004, "ensemble attitude error exceeds 0.004 rad");
    require(bias_rms < 0.00015, "ensemble bias estimate did not converge");
    // The expected mean is six for a perfectly calibrated six-state Gaussian
    // model. This broad regression envelope catches gross over/underconfidence;
    // it is not a statistical certification of nonlinear filter consistency.
    require(mean_nees > 1 && mean_nees < 18, "ensemble covariance is grossly miscalibrated");
}
} // namespace

int main() {
    unsigned failures = 0;
    auto test = [&](const char *name, const std::function<void()> &body) {
        try {
            body();
            std::cout << "PASS " << name << '\n';
        } catch (const std::exception &e) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << e.what() << '\n';
        }
    };
    test("quaternion conventions and sign ambiguity", conventions);
    test("reset and transition finite-difference Jacobians", jacobians);
    test("closed-form covariance and composition", process_noise);
    test("outlier rejection and input validation", gating_and_errors);
    test("24 seeded tracker-outage simulations", monte_carlo);
    return failures == 0 ? 0 : 1;
}
