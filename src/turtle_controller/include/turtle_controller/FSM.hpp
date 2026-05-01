#include <algorithm>
#include "chrono"
#include "cstdio"

enum class State
{
    ESCAPE,
    TRANSITION,
    EXPLORE
};

class FSM
{
public:
    FSM();
    State update(double global_risk_);
    State getCurrentState();

private:
    State current_state = State::EXPLORE;
    State computeState(double global_risk_);
    std::chrono::steady_clock::time_point start_enter_state;
    double min_time_in_state = 0.3; // seconds
};