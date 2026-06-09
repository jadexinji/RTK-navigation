#include "coordinate/CoordinateTransformer.h"

#include <cmath>
#include <stdexcept>

namespace rtk_nav {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84SemiMajorAxis = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kWgs84FirstEccentricitySquared =
    kWgs84Flattening * (2.0 - kWgs84Flattening);

double degToRad(double degrees) {
    return degrees * kPi / 180.0;
}

}  // namespace

CoordinateTransformer::CoordinateTransformer(double origin_latitude,
                                             double origin_longitude,
                                             double origin_height) {
    setOrigin(origin_latitude, origin_longitude, origin_height);
}

void CoordinateTransformer::setOrigin(double latitude, double longitude, double height) {
    origin_latitude_ = latitude;
    origin_longitude_ = longitude;
    origin_height_ = height;
    origin_ecef_ = geodeticToEcef(latitude, longitude, height);

    const double lat_rad = degToRad(latitude);
    const double lon_rad = degToRad(longitude);
    const double sin_lat = std::sin(lat_rad);
    const double cos_lat = std::cos(lat_rad);
    const double sin_lon = std::sin(lon_rad);
    const double cos_lon = std::cos(lon_rad);

    ecef_to_enu_ << -sin_lon, cos_lon, 0.0,
                    -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
                     cos_lat * cos_lon,  cos_lat * sin_lon, sin_lat;

    has_origin_ = true;
}

LocalPoint CoordinateTransformer::toLocal(const RTKPoint& point) const {
    if (!has_origin_) {
        throw std::runtime_error("CoordinateTransformer origin has not been set");
    }

    const Eigen::Vector3d point_ecef = geodeticToEcef(point.latitude, point.longitude, point.height);
    const Eigen::Vector3d enu = ecef_to_enu_ * (point_ecef - origin_ecef_);

    LocalPoint local;
    local.id = point.id;
    local.x = enu.x();
    local.y = enu.y();
    local.z = enu.z();
    local.type = point.type;
    return local;
}

std::vector<LocalPoint> CoordinateTransformer::toLocal(const std::vector<RTKPoint>& points) const {
    std::vector<LocalPoint> local_points;
    local_points.reserve(points.size());
    for (const RTKPoint& point : points) {
        local_points.push_back(toLocal(point));
    }
    return local_points;
}

Eigen::Vector3d CoordinateTransformer::geodeticToEcef(double latitude,
                                                       double longitude,
                                                       double height) const {
    const double lat_rad = degToRad(latitude);
    const double lon_rad = degToRad(longitude);
    const double sin_lat = std::sin(lat_rad);
    const double cos_lat = std::cos(lat_rad);
    const double sin_lon = std::sin(lon_rad);
    const double cos_lon = std::cos(lon_rad);

    const double prime_vertical_radius =
        kWgs84SemiMajorAxis /
        std::sqrt(1.0 - kWgs84FirstEccentricitySquared * sin_lat * sin_lat);

    const double x = (prime_vertical_radius + height) * cos_lat * cos_lon;
    const double y = (prime_vertical_radius + height) * cos_lat * sin_lon;
    const double z = (prime_vertical_radius * (1.0 - kWgs84FirstEccentricitySquared) + height) * sin_lat;

    return Eigen::Vector3d{x, y, z};
}

}  // namespace rtk_nav
