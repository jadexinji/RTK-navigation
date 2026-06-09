#pragma once

#include <vector>

#include <Eigen/Dense>

#include "common/Types.h"

namespace rtk_nav {

class CoordinateTransformer {
public:
    CoordinateTransformer() = default;
    CoordinateTransformer(double origin_latitude, double origin_longitude, double origin_height);

    void setOrigin(double latitude, double longitude, double height);

    LocalPoint toLocal(const RTKPoint& point) const;
    std::vector<LocalPoint> toLocal(const std::vector<RTKPoint>& points) const;

    double originLatitude() const { return origin_latitude_; }
    double originLongitude() const { return origin_longitude_; }
    double originHeight() const { return origin_height_; }

private:
    Eigen::Vector3d geodeticToEcef(double latitude, double longitude, double height) const;

    bool has_origin_ = false;
    double origin_latitude_ = 0.0;
    double origin_longitude_ = 0.0;
    double origin_height_ = 0.0;
    Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d ecef_to_enu_ = Eigen::Matrix3d::Identity();
};

}  // namespace rtk_nav
