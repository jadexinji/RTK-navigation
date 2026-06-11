#pragma once

#include <string>
#include <vector>

#include "common/Types.h"

namespace rtk_nav {

struct CassPoint {
    std::string id;
    std::string code;
    double easting = 0.0;
    double northing = 0.0;
    double height = 0.0;
};

struct CassLocalPoint {
    LocalPoint point;
    std::string group;
};

class CassDatReader {
public:
    std::vector<CassPoint> load(const std::string& dat_path) const;
    std::vector<CassPoint> filterSurveyArea(const std::vector<CassPoint>& points,
                                            double max_distance_from_median = 500.0) const;
    std::vector<CassLocalPoint> toLocal(const std::vector<CassPoint>& points,
                                        double origin_easting,
                                        double origin_northing) const;

    static std::string pointGroup(const std::string& point_id);
};

}  // namespace rtk_nav
