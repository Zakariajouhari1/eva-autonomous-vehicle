// osrm_client.hpp
// Create this file at: include/eva_planning/osrm_client.hpp

#ifndef EVA_PLANNING_OSRM_CLIENT_HPP
#define EVA_PLANNING_OSRM_CLIENT_HPP

#include <string>
#include <vector>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <iostream>

namespace eva_planning {

struct RouteWaypoint {
    double latitude;
    double longitude;
    std::string instruction;
    double distance;  // meters to next point
    std::string street_name;
};

struct RouteInfo {
    std::vector<RouteWaypoint> waypoints;
    double total_distance;  // meters
    double total_duration;  // seconds
};

class OSRMClient {
public:
    OSRMClient(const std::string& server_url = "http://router.project-osrm.org")
        : server_url_(server_url) {
        curl_global_init(CURL_GLOBAL_DEFAULT);
    }
    
    ~OSRMClient() {
        curl_global_cleanup();
    }
    
    RouteInfo getRoute(double start_lat, double start_lon,
                      double end_lat, double end_lon) {
        std::string url = buildURL(start_lat, start_lon, end_lat, end_lon);
        std::string response = makeRequest(url);
        return parseResponse(response);
    }
    
    void setServerURL(const std::string& url) {
        server_url_ = url;
    }
    
private:
    std::string server_url_;
    
    std::string buildURL(double start_lat, double start_lon,
                        double end_lat, double end_lon) {
        // OSRM format: /route/v1/driving/{lon,lat;lon,lat}
        return server_url_ + "/route/v1/driving/" +
               std::to_string(start_lon) + "," + std::to_string(start_lat) + ";" +
               std::to_string(end_lon) + "," + std::to_string(end_lat) +
               "?steps=true&geometries=geojson&overview=full";
    }
    
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
        ((std::string*)userp)->append((char*)contents, size * nmemb);
        return size * nmemb;
    }
    
    std::string makeRequest(const std::string& url) {
        CURL* curl = curl_easy_init();
        std::string response;
        
        if (curl) {
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
            
            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "[OSRM] CURL error: " << curl_easy_strerror(res) << std::endl;
            }
            curl_easy_cleanup(curl);
        }
        
        return response;
    }
    
    RouteInfo parseResponse(const std::string& response) {
        RouteInfo route;
        
        try {
            auto j = nlohmann::json::parse(response);
            
            // Check if route was successful
            if (!j.contains("code") || j["code"] != "Ok") {
                std::cerr << "[OSRM] Route request failed: " 
                         << j.value("code", "unknown") << std::endl;
                return route;
            }
            
            if (j.contains("routes") && !j["routes"].empty()) {
                auto& r = j["routes"][0];
                route.total_distance = r.value("distance", 0.0);
                route.total_duration = r.value("duration", 0.0);
                
                if (r.contains("legs") && !r["legs"].empty()) {
                    auto& leg = r["legs"][0];
                    
                    if (leg.contains("steps")) {
                        for (auto& step : leg["steps"]) {
                            if (step.contains("geometry") && 
                                step["geometry"].contains("coordinates")) {
                                
                                auto& coords = step["geometry"]["coordinates"];
                                
                                // Add all coordinates from this step
                                for (auto& coord : coords) {
                                    RouteWaypoint wp;
                                    wp.longitude = coord[0].get<double>();
                                    wp.latitude = coord[1].get<double>();
                                    wp.distance = step.value("distance", 0.0) / coords.size();
                                    wp.street_name = step.value("name", "");
                                    
                                    if (step.contains("maneuver") && 
                                        step["maneuver"].contains("instruction")) {
                                        wp.instruction = step["maneuver"]["instruction"];
                                    }
                                    
                                    route.waypoints.push_back(wp);
                                }
                            }
                        }
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[OSRM] JSON parsing error: " << e.what() << std::endl;
        }
        
        return route;
    }
};

} // namespace eva_planning

#endif // EVA_PLANNING_OSRM_CLIENT_HPP
