#include "turtle_controller/FSM.hpp"
constexpr double border_hysteresis = 0.15;

FSM::FSM()
{
    start_enter_state = std::chrono::steady_clock::now();
};

State FSM::getCurrentState()
{
    return current_state;
};

State FSM::update(double global_risk_)
{

    auto now = std::chrono::steady_clock::now();

    double time_in_state =
        std::chrono::duration<double>(now - start_enter_state).count();

    State candidate = computeState(global_risk_);
    if (candidate == State::ESCAPE)
    {
        if (current_state != State::ESCAPE)
        {
            current_state = State::ESCAPE;
            start_enter_state = now;
        }
        return current_state;
    }

    if (time_in_state < min_time_in_state)
    {
        return current_state;
    }

    // cambio stato
    if (candidate != current_state)
    {
        current_state = candidate;
        start_enter_state = now;
    }

    return current_state;
};
State FSM::computeState(double risk)
{
    switch (current_state)
    {
    case State::EXPLORE:
        if (risk > 0.40)
            return State::TRANSITION;
        return State::EXPLORE;

    case State::TRANSITION:
        if (risk > 0.60)
            return State::ESCAPE;

        if (risk < 0.30)
            return State::EXPLORE;

        return State::TRANSITION;

    case State::ESCAPE:
        if (risk < 0.45)
            return State::TRANSITION;

        if (risk < 0.20)
            return State::EXPLORE;

        return State::ESCAPE;
    }

    return State::EXPLORE;
}
