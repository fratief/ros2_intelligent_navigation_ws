#pragma once
#include <random>

struct ReactivePlannerOutput
{
    double linear_speed{0.0};
    double angular_speed{0.0};
};

struct Risks
{
    double front_risk{0.0};
    double front_wide_risk{0.0};
    double left_risk{0.0};
    double right_risk{0.0};
    double back_risk{0.0};
    double global_risk{0.0};
};

class ReactivePlanner
{
public:
    ReactivePlanner();

    void updateRisks(const Risks& risks);

    ReactivePlannerOutput handle_escape();
    ReactivePlannerOutput handle_transition();
    ReactivePlannerOutput handle_explore(double current_time_sec);

private:
    double chooseEscapeDirection();

    Risks risks_;

    double previous_turn_{0.0};
    double last_explore_change_time_{0.0};
    double min_duration_explore_{1.5};
    double explore_k_weight_{0.8};
    double explore_turn_{0.0};
    double escape_k_mult{1.5};
    double transition_k_random_{0.10};

    static constexpr double X_SPEED = 0.30;
    static constexpr double X_SPEED_TRANSITION = 0.22;
    static constexpr double X_SPEED_ESCAPE = 0.16;

    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<double> dist;
};