#include "attitude/filter.hpp"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>

int main(int argc, char **argv) {
    using namespace attitude;
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") {
            std::cout
                << "Usage: attitude-demo [SEED]\n"
                   "A 300-second gyro/star-tracker experiment. CSV to stdout; summary to stderr.\n"
                   "Tracker outage: 80-120 s. One false observation: 150 s.\n";
            return 0;
        }
        if (argc > 2)
            throw std::invalid_argument("expected at most one seed");
        std::uint64_t seed = 42;
        if (argc == 2) {
            const std::string value(argv[1]);
            const auto [end, error] =
                std::from_chars(value.data(), value.data() + value.size(), seed);
            if (error != std::errc{} || end != value.data() + value.size())
                throw std::invalid_argument("invalid seed");
        }
        std::mt19937_64 engine(seed);
        std::normal_distribution<double> normal;
        auto random_vector = [&]() { return Vec3{normal(engine), normal(engine), normal(engine)}; };
        constexpr double dt = 0.2;
        constexpr double tracker_std = 0.002;
        constexpr double to_degrees = 180 / std::numbers::pi;
        const Config config;
        Quaternion truth = Quaternion::Identity();
        Quaternion gyro_only = rotation_exp({0.08, -0.06, 0.04});
        Vec3 true_bias{0.001, -0.0015, 0.0008};
        Filter filter(gyro_only, config);
        std::size_t accepted = 0, rejected = 0, missing = 0;
        double maximum_dropout_error = 0;
        std::cout << std::setprecision(12)
                  << "time_s,angle_error_deg,gyro_only_error_deg,bias_error_rad_s,angle_sigma_deg,"
                     "tracker_status\n";
        for (int step = 1; step <= 1500; ++step) {
            const double time = step * dt;
            const Vec3 rate{0.012 * std::sin(time / 30), 0.009 * std::cos(time / 23), 0.018};
            truth = (truth * Quaternion(Eigen::AngleAxisd(rate.norm() * dt, rate.normalized())))
                        .normalized();
            const Vec3 gyro =
                rate + true_bias + random_vector() * (config.noise.gyro / std::sqrt(dt));
            true_bias += random_vector() * (config.noise.bias_walk * std::sqrt(dt));
            gyro_only = (gyro_only * rotation_exp(gyro * dt)).normalized();
            filter.propagate(gyro, dt);
            const bool outage = time >= 80 && time < 120;
            const char *status = "none";
            if (step % 5 == 0) {
                if (outage) {
                    ++missing;
                    status = "missing";
                } else {
                    Quaternion tracker = truth * rotation_exp(random_vector() * tracker_std);
                    if (step == 750)
                        tracker = tracker * rotation_exp({0.6, 0, 0});
                    if (step % 10 == 0)
                        tracker.coeffs() *= -1;
                    const auto result =
                        filter.observe(tracker, Mat3::Identity() * tracker_std * tracker_std);
                    if (result.accepted) {
                        ++accepted;
                        status = "accepted";
                    } else {
                        ++rejected;
                        status = "rejected";
                    }
                }
            }
            const auto &estimate = filter.estimate();
            const double error =
                rotation_log(estimate.orientation.conjugate() * truth).norm() * to_degrees;
            if (outage)
                maximum_dropout_error = std::max(maximum_dropout_error, error);
            if (step % 5 == 0) {
                std::cout << time << ',' << error << ','
                          << rotation_log(gyro_only.conjugate() * truth).norm() * to_degrees << ','
                          << (true_bias - estimate.bias).norm() << ','
                          << std::sqrt(estimate.covariance.topLeftCorner<3, 3>().trace() / 3) *
                                 to_degrees
                          << ',' << status << '\n';
            }
        }
        const auto &result = filter.estimate();
        std::cerr << std::setprecision(12) << "{\"seed\":" << seed << ",\"final_angle_error_deg\":"
                  << rotation_log(result.orientation.conjugate() * truth).norm() * to_degrees
                  << ",\"gyro_only_error_deg\":"
                  << rotation_log(gyro_only.conjugate() * truth).norm() * to_degrees
                  << ",\"bias_error_rad_s\":" << (true_bias - result.bias).norm()
                  << ",\"max_dropout_error_deg\":" << maximum_dropout_error
                  << ",\"accepted\":" << accepted << ",\"rejected\":" << rejected
                  << ",\"missing\":" << missing << "}\n";
        if (!std::cout)
            throw std::runtime_error("failed writing output");
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "attitude-demo: " << e.what() << '\n';
        return 1;
    }
}
