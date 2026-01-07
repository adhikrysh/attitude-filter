#include "attitude/filter.hpp"

#include <cmath>
#include <stdexcept>

namespace attitude {
Mat3 skew(const Vec3 &v) {
    Mat3 result;
    result << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return result;
}

Quaternion rotation_exp(const Vec3 &vector) {
    if (!vector.allFinite())
        throw std::invalid_argument("rotation vector must be finite");
    const double angle = vector.norm();
    if (!std::isfinite(angle))
        throw std::invalid_argument("rotation magnitude overflow");
    const double scale = angle < 1e-8 ? 0.5 - angle * angle / 48 : std::sin(angle / 2) / angle;
    return Quaternion(std::cos(angle / 2), scale * vector.x(), scale * vector.y(),
                      scale * vector.z());
}

Vec3 rotation_log(Quaternion q) {
    const double length = q.norm();
    if (!q.coeffs().allFinite() || !(length > 1e-12) || !std::isfinite(length)) {
        throw std::invalid_argument("invalid quaternion");
    }
    q.normalize();
    // q and -q describe the same attitude. Using the shorter rotation also
    // prevents a tracker's harmless sign flip from appearing as a large error.
    if (q.w() < 0)
        q.coeffs() *= -1;
    const double sine = q.vec().norm();
    if (sine < 1e-10)
        return 2 * q.vec();
    return q.vec() * (2 * std::atan2(sine, q.w()) / sine);
}

Mat3 right_jacobian(const Vec3 &vector) {
    if (!vector.allFinite())
        throw std::invalid_argument("rotation vector must be finite");
    const double angle = vector.norm();
    if (!std::isfinite(angle))
        throw std::invalid_argument("rotation magnitude overflow");
    const Mat3 cross = skew(vector);
    if (angle < 1e-5) {
        const double squared = angle * angle;
        return Mat3::Identity() - (0.5 - squared / 24) * cross +
               (1.0 / 6 - squared / 120) * cross * cross;
    }
    return Mat3::Identity() - ((1 - std::cos(angle)) / (angle * angle)) * cross +
           ((angle - std::sin(angle)) / (angle * angle * angle)) * cross * cross;
}
} // namespace attitude
