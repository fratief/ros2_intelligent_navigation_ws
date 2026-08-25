#pragma once
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"

struct Slice {
    int start_idx{0};
    int end_idx{0};
};

struct SectorIndices {
    std::vector<Slice> slices;
};

class RiskModel
{
public:
    explicit RiskModel(double alpha = 0.45);
    ~RiskModel();

    void updateScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan);
    void update();

    double getGlobalRisk() const;
    double getFrontRisk() const;
    double getFrontWideRisk() const;
    double getLeftRisk() const;
    double getRightRisk() const;
    double getBackRisk() const;

private:
    int angleToIndex(double target_angle, double angle_min, double angle_inc, int max_size) const;
    SectorIndices createSector(double start_angle, double end_angle, double angle_min, double angle_inc, int max_size) const;
    void buildSectorCache(double angle_min, double angle_inc, int size);
    
    double processSector(const std::vector<float>& ranges, const SectorIndices& sector, float range_min, float range_max) const;

    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    double alpha_;
    double global_risk_{0.0};
    double front_risk_{0.0};
    double front_wide_risk_{0.0};
    double left_risk_{0.0};
    double right_risk_{0.0};
    double back_risk_{0.0};
    bool isInitialize{false};

    // Cache degli indici
    SectorIndices front_indices_;
    SectorIndices front_wide_indices_;
    SectorIndices left_indices_;
    SectorIndices right_indices_;
    SectorIndices back_indices_;

    bool indices_initialized_{false};
    int cached_scan_size_{0};
};