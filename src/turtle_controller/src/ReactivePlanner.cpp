#include "turtle_controller/ReactivePlanner.hpp"

ReactivePlanner::ReactivePlanner()
    : gen(rd()), dist(-1.0, 1.0)
{
}

void ReactivePlanner::updateRisks(const Risks& risks)
{
    risks_ = risks;
}

ReactivePlannerOutput ReactivePlanner::handle_escape()
{
    ReactivePlannerOutput output;
    double dir = chooseEscapeDirection();
    double repulsion = risks_.right_risk - risks_.left_risk;

    double turn = 0.7 * dir + 0.3 * repulsion;

    output.linear_speed = X_SPEED_ESCAPE * (1.0 - risks_.global_risk);
    output.angular_speed = turn * escape_k_mult;

    previous_turn_ = turn;
    return output;
}

ReactivePlannerOutput ReactivePlanner::handle_transition()
{
    ReactivePlannerOutput output;

    double dir = chooseEscapeDirection();
    double repulsion = risks_.right_risk - risks_.left_risk;
    double explore_bias = dist(gen);

    double turn =
        0.35 * dir +
        0.20 * repulsion +
        0.35 * previous_turn_ +
        transition_k_random_ * explore_bias;

    output.linear_speed = X_SPEED_TRANSITION * (1.0 - risks_.global_risk);
    output.angular_speed = turn * (1.0 - risks_.global_risk);

    previous_turn_ = turn;
    return output;
}

ReactivePlannerOutput ReactivePlanner::handle_explore(double current_time_sec)
{
    ReactivePlannerOutput output;

    if ((current_time_sec - last_explore_change_time_) > min_duration_explore_)
    {
        last_explore_change_time_ = current_time_sec;
        explore_turn_ = dist(gen) * 0.6;
    }

    double turn = explore_k_weight_ * explore_turn_ + 0.2 * previous_turn_;

    output.linear_speed = X_SPEED * (1.0 - risks_.global_risk);
    output.angular_speed = turn * (1.0 - risks_.global_risk);

    previous_turn_ = turn;
    return output;
}

double ReactivePlanner::chooseEscapeDirection()
{
    if (risks_.front_risk > 0.25)
    {
        if (risks_.left_risk < risks_.right_risk)
            return 1.0;
        else
            return -1.0;
    }

    if (risks_.front_wide_risk > 0.20)
    {
        if (risks_.left_risk < risks_.right_risk)
            return 0.7;
        else
            return -0.7;
    }

    if (risks_.left_risk < risks_.right_risk)
        return 0.5;
    else if (risks_.right_risk < risks_.left_risk)
        return -0.5;

    return previous_turn_ >= 0 ? 0.5 : -0.5;
}