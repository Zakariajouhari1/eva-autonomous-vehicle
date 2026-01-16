// coordinate_converter.hpp
// Create this file at: include/eva_planning/coordinate_converter.hpp

#ifndef EVA_PLANNING_COORDINATE_CONVERTER_HPP
#define EVA_PLANNING_COORDINATE_CONVERTER_HPP

#include <cmath>
#include <utility>

namespace eva_planning {

class CoordinateConverter {
public:
    // Default origin: Casablanca, Morocco
    CoordinateConverter(double origin_lat = 33.5731, double origin_lon = -7.5898)
        : origin_lat_(origin_lat), origin_lon_(origin_lon) {
        updateMetersPerDegree();
    }
    
    // Convert GPS coordinates to local odometry frame (meters)
    std::pair<double, double> gpsToOdom(double lat, double lon) const {
        double x = (lon - origin_lon_) * meters_per_degree_lon_;
        double y = (lat - origin_lat_) * meters_per_degree_lat_;
        return {x, y};
    }
    
    // Convert local odometry coordinates to GPS
    std::pair<double, double> odomToGps(double x, double y) const {
        double lat = origin_lat_ + (y / meters_per_degree_lat_);
        double lon = origin_lon_ + (x / meters_per_degree_lon_);
        return {lat, lon};
    }
    
    // Set new origin point
    void setOrigin(double lat, double lon) {
        origin_lat_ = lat;
        origin_lon_ = lon;
        updateMetersPerDegree();
    }
    
    // Get current origin
    std::pair<double, double> getOrigin() const {
        return {origin_lat_, origin_lon_};
    }

private:
    double origin_lat_;
    double origin_lon_;
    double meters_per_degree_lat_;
    double meters_per_degree_lon_;
    
    void updateMetersPerDegree() {
        // Approximately 111,320 meters per degree latitude
        meters_per_degree_lat_ = 111320.0;
        // Longitude varies with latitude
        meters_per_degree_lon_ = 111320.0 * std::cos(origin_lat_ * M_PI / 180.0);
    }
};

} // namespace eva_planning

#endif // EVA_PLANNING_COORDINATE_CONVERTER_HPP
