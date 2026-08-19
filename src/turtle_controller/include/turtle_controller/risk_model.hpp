#pragma once
#include <vector>
#include <memory>
#include "sensor_msgs/msg/laser_scan.hpp"

class RiskModel
{
private:
    // --- Struct per raccogliere i dati del settore --- (UPGRADE FUTURO)
    struct SectorData
    {
        double weighted_sum = 0.0;
        double weight_total = 0.0;
        double min_dist = 1.0;
        int count_close = 0;
        int count_total = 0;
    };

    // --- Parametri ---
    double alpha_; // smoothing

    // --- Rischi smussati ---
    double global_risk_;
    double front_risk_;
    double front_wide_risk_;
    double left_risk_;
    double right_risk_;
    double back_risk_;

    // --- Angoli dei settori (radianti) ---
    double front_start_, front_end_;
    double front_wide_start_, front_wide_end_;
    double left_start_, left_end_;
    double right_start_, right_end_;
    double back_start_, back_end_;

    // check
    bool isInitialize;

    // Ultimo LaserScan
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    // --- Funzione principale ---
    double getRiskBySector(double start_angle, double end_angle) const;

    /*
    // --- Sottofunzioni ---
    SectorData collectSectorData(double start_angle, double end_angle) const;

    double computeMeanRisk(const SectorData& d) const;
    double computeMaxRisk(const SectorData& d) const;
    double computeDensityRisk(const SectorData& d) const;
    double combineRisks(double mean, double max, double density) const;

    // --- Utility ---
    bool angleInSector(double angle, double start, double end) const;
    bool isValidRange(float r) const;
    double computeLocalRisk(double rc, double safe, double min_d) const;
    double normalizeAngle(double angle) const;
    double smooth(double previous, double current) const;
    */

    double normalizeAngle(double a) const;

public:
    RiskModel(double alpha = 0.75);
    ~RiskModel();

    // Aggiorna lo scan
    void updateScan(sensor_msgs::msg::LaserScan::SharedPtr &scan);

    // Calcola tutti i rischi
    void update();

    // Getter
    double getGlobalRisk() const;
    double getFrontRisk() const;
    double getFrontWideRisk() const;
    double getLeftRisk() const;
    double getRightRisk() const;
    double getBackRisk() const;
};
