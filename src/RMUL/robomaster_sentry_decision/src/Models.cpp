#include "sentry_decision/Models.hpp"
#include <cmath>
#include <algorithm>

double Models::calculateSituationZ(const Blackboard& bb) {
    double hp_ratio = bb.current_hp / 400.0;
    double ammo_ratio = bb.allowance_17mm / 750.0;
    return 1.0 * hp_ratio + 0.0 * ammo_ratio;
}
