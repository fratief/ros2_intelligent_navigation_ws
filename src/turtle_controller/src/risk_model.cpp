#include "turtle_controller/risk_model.hpp"

RiskModel::RiskModel(double alpha)
    : alpha_(alpha)
{
}

RiskModel::~RiskModel() {}

void RiskModel::updateScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
{
    last_scan_ = scan;
}

int RiskModel::angleToIndex(double target_angle, double angle_min, double angle_inc, int max_size) const
{
    // formula per trovare indice Message laser_scan, Delta angoloTarger e angolo minimo scansione normalizzato tra 0 e 2PI diviso per incremento angolare messaggio LaserScan
    double diff = target_angle - angle_min;
    while (diff < 0.0) diff += 2.0 * M_PI;
    while (diff >= 2.0 * M_PI) diff -= 2.0 * M_PI;

    int idx = static_cast<int>(diff / angle_inc);
    return std::clamp(idx, 0, max_size - 1);
}

SectorIndices RiskModel::createSector(double start_angle, double end_angle, double angle_min, double angle_inc, int max_size) const
{
    // creazione Indice per settore, considerando il wrap-around a cavallo del limite dell'array
    int start_idx = angleToIndex(start_angle, angle_min, angle_inc, max_size);
    int end_idx = angleToIndex(end_angle, angle_min, angle_inc, max_size);

    SectorIndices sector;
    if (start_idx <= end_idx) {
        sector.slices.push_back({start_idx, end_idx});
    } else {
        // Gestione del wrap-around a cavallo del limite dell'array. Esempio: se start_idx = 350 e end_idx = 10, significa che il settore attraversa l'indice 0. Quindi creiamo due fette: una da start_idx a max_size - 1 e un'altra da 0 a end_idx.
        sector.slices.push_back({start_idx, max_size - 1});
        sector.slices.push_back({0, end_idx});
    }
    return sector;
}

void RiskModel::buildSectorCache(double angle_min, double angle_inc, int size)
{
    // Creo gli indici per ciascun settore in base agli angoli specificati
    front_indices_      = createSector(-M_PI / 6.0,  M_PI / 6.0, angle_min, angle_inc, size);
    front_wide_indices_ = createSector(-M_PI / 3.0,  M_PI / 3.0, angle_min, angle_inc, size);
    left_indices_       = createSector( M_PI / 6.0,  M_PI / 3.0, angle_min, angle_inc, size);
    right_indices_      = createSector(-M_PI / 3.0, -M_PI / 6.0, angle_min, angle_inc, size);
    back_indices_       = createSector( 5.0 * M_PI / 6.0, -5.0 * M_PI / 6.0, angle_min, angle_inc, size);

    indices_initialized_ = true;
    cached_scan_size_ = size;
}

double RiskModel::processSector(const std::vector<float>& ranges, const SectorIndices& sector, float range_min, float range_max) const
{
    constexpr double safe = 1.0;
    constexpr double min_d = 0.15;
    constexpr double density_threshold = 0.50;

    double weighted_sum = 0.0;
    double weight_total = 0.0;
    double min_dist = safe;
    int count_close = 0;
    int count_total = 0;

    // Iterazione diretta SOLO sugli indici appartenenti alle fette del settore, evitando cicli su tutta la scansione. Questo riduce significativamente il numero di iterazioni e migliora le prestazioni.
    for (const auto& slice : sector.slices)
    {
        for (int i = slice.start_idx; i <= slice.end_idx; ++i)
        {
            float r = ranges[i];
            if (!std::isfinite(r) || r <= 0.0f || r < range_min || r > range_max)
                continue;

            double rc = std::clamp(static_cast<double>(r), min_d, safe);
            min_dist = std::min(min_dist, rc);

            double w = 1.0 / rc;
            double local_risk = (safe - rc) / (safe - min_d);

            weighted_sum += w * local_risk;
            weight_total += w;
            count_total++;

            if (rc < density_threshold)
                count_close++;
        }
    }

    if (count_total == 0)
        return 0.0;

    double risk_mean = weighted_sum / weight_total;
    double risk_max = (safe - min_dist) / (safe - min_d);
    double density = static_cast<double>(count_close) / count_total;

    double risk_sector = 0.70 * risk_max + 0.25 * density + 0.05 * risk_mean;
    return std::clamp(risk_sector, 0.0, 1.0);
}

void RiskModel::update()
{
    if (!last_scan_)
        return;

    const auto &ranges = last_scan_->ranges;
    int size = ranges.size();
    if (size == 0)
        return;

    // Ricalcola la mappa degli indici solo se necessario
    if (!indices_initialized_ || cached_scan_size_ != size)
    {
        buildSectorCache(last_scan_->angle_min, last_scan_->angle_increment, size);
    }

    // Calcolo istantaneo del rischio per ciascun settore senza cicli su tutta la scansione
    front_risk_      = processSector(ranges, front_indices_, last_scan_->range_min, last_scan_->range_max);
    front_wide_risk_ = processSector(ranges, front_wide_indices_, last_scan_->range_min, last_scan_->range_max);
    left_risk_       = processSector(ranges, left_indices_, last_scan_->range_min, last_scan_->range_max);
    right_risk_      = processSector(ranges, right_indices_, last_scan_->range_min, last_scan_->range_max);
    back_risk_       = processSector(ranges, back_indices_, last_scan_->range_min, last_scan_->range_max);

    // Integrazione del Rischio Globale e Filtro Passa-Basso
    double weighted_risk =
        0.60 * front_risk_ +
        0.30 * front_wide_risk_ +
        0.035 * left_risk_ +
        0.035 * right_risk_ +
        0.03 * back_risk_;

    double max_sector = std::max({front_risk_, front_wide_risk_, left_risk_, right_risk_, back_risk_});
    weighted_risk = std::max(weighted_risk, 0.70 * max_sector);

    double alpha = (weighted_risk > global_risk_) ? 0.45 : 0.80;

    if (!isInitialize)
    {
        global_risk_ = weighted_risk;
        isInitialize = true;
    }
    else
    {
        global_risk_ = alpha * global_risk_ + (1.0 - alpha) * weighted_risk;
    }
}

double RiskModel::getGlobalRisk() const { return global_risk_; }
double RiskModel::getFrontRisk() const { return front_risk_; }
double RiskModel::getFrontWideRisk() const { return front_wide_risk_; }
double RiskModel::getLeftRisk() const { return left_risk_; }
double RiskModel::getRightRisk() const { return right_risk_; }
double RiskModel::getBackRisk() const { return back_risk_; }