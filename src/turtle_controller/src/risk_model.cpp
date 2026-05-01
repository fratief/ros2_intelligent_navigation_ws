#include "turtle_controller/risk_model.hpp"
#include <algorithm>
#include <cmath>
// public functions
RiskModel::RiskModel(double alpha) : alpha_(alpha), global_risk_(0.0), front_risk_(0.0), front_wide_risk_(0.0), left_risk_(0.0), right_risk_(0.0), back_risk_(0.0)
{
    front_start_ = -M_PI / 6;
    front_end_ = M_PI / 6;
    front_wide_start_ = -M_PI / 3;
    front_wide_end_ = M_PI / 3;
    right_start_ = -M_PI / 3;
    right_end_ = -M_PI / 6;
    left_start_ = M_PI / 6;
    left_end_ = M_PI / 3;
    back_start_ = 5 * M_PI / 6;
    back_end_ = 7 * M_PI / 6;
    isInitialize = false;
};

RiskModel::~RiskModel() {};

void RiskModel::updateScan(sensor_msgs::msg::LaserScan::SharedPtr &scan)
{
    last_scan_ = scan;
}

void RiskModel::update()
{
    left_risk_ = getRiskBySector(left_start_, left_end_);    // +30° → +60°
    right_risk_ = getRiskBySector(right_start_, right_end_); // -60° → -30°

    front_risk_ = getRiskBySector(front_start_, front_end_);                // ±30°
    front_wide_risk_ = getRiskBySector(front_wide_start_, front_wide_end_); // ±60°
    back_risk_ = getRiskBySector(
        back_start_, // 150°
        back_end_    // 210°
    );

    double weighted_risk =
        0.50 * front_risk_        // ostacoli diretti davanti
        + 0.25 * front_wide_risk_ // corridoi, muri laterali, ambienti stretti
        + 0.1 * left_risk_        // per scegliere direzione di fuga
        + 0.1 * right_risk_       // idem
        + 0.05 * back_risk_;      // evita di girarsi verso un muro

    double alpha = 0.75; // più alto = più stabile

    if (!isInitialize)
    {
        global_risk_ = weighted_risk;
        isInitialize = true;
    }
    else
    {
        global_risk_ = alpha * global_risk_ + (1 - alpha) * weighted_risk;
    }
}

double RiskModel::getGlobalRisk() const
{
    return global_risk_;
}
double RiskModel::getFrontRisk() const
{
    return front_risk_;
};
double RiskModel::getFrontWideRisk() const
{
    return front_wide_risk_;
};

double RiskModel::getLeftRisk() const
{
    return left_risk_;
};

double RiskModel::getRightRisk() const
{
    return right_risk_;
};
double RiskModel::getBackRisk() const
{
    return back_risk_;
};

// private

double RiskModel::getRiskBySector(double start_angle, double end_angle) const
{
    if (!last_scan_)
        return 0.0;

    const auto &ranges = last_scan_->ranges;
    int size = ranges.size();
    if (size == 0)
        return 0.0;

    double angle_min = last_scan_->angle_min;
    double angle_inc = last_scan_->angle_increment;
    start_angle = normalizeAngle(start_angle);
    end_angle = normalizeAngle(end_angle);

    bool wrap = (start_angle > end_angle);
    double safe = 1.0;
    double min_d = 0.15;
    double density_threshold = 0.50;

    double weighted_sum = 0.0;
    double weight_total = 0.0;
    double min_dist = safe; // valore sicuro

    int count_close = 0;
    int count_total = 0;

    for (int i = 0; i < size; ++i)
    {
        double angle = angle_min + i * angle_inc;
        bool in_sector =
            (!wrap && angle >= start_angle && angle <= end_angle) ||
            (wrap && (angle >= start_angle || angle <= end_angle));

        if (!in_sector)
            continue;

        float r = ranges[i];

        // FILTRO VALORI INVALIDI
        if (!std::isfinite(r))
            continue;
        if (r <= 0.0)
            continue;
        if (r < last_scan_->range_min)
            continue;
        if (r > last_scan_->range_max)
            continue;

        // clamp
        double rc = std::clamp((double)r, min_d, safe);

        // min distance
        min_dist = std::min(min_dist, rc);

        // weighted mean
        double w = 1.0 / rc;
        double local_risk = (safe - rc) / (safe - min_d);

        weighted_sum += w * local_risk;
        weight_total += w;

        // density
        count_total++;
        if (rc < density_threshold)
            count_close++;
    }

    if (count_total == 0)
        return 0.0; // settore vuoto ---> rischio zero

    double risk_mean = weighted_sum / weight_total;

    double risk_max = (safe - min_dist) / (safe - min_d);

    double density = (double)count_close / count_total;

    double risk_sector =
        0.6 * risk_mean + 0.3 * risk_max + 0.1 * density;

    return std::clamp(risk_sector, 0.0, 1.0);
};

double RiskModel::normalizeAngle(double a) const
{
    double two_pi = 2.0 * M_PI;
    while (a < 0.0)
        a += two_pi;
    while (a >= two_pi)
        a -= two_pi;
    return a;
}
