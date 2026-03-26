#include "sentry_decision/Models.hpp"
#include <cmath>
#include <algorithm>

double Models::calculateSituationZ(const Blackboard& bb) {
    double hp_ratio = bb.current_hp / 400.0;
    double ammo_ratio = bb.allowance_17mm / 750.0;
    double hp_weight = bb.getHpWeight();
    double ammo_weight = bb.getAmmoWeight();
    return hp_weight * hp_ratio + ammo_weight * ammo_ratio;
}
