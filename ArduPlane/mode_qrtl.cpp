#include "mode.h"
#include "Plane.h"

bool ModeQRTL::_enter()
{
    plane.mode_qstabilize._enter();
    // QRTL starts off in QPOS_APPROACH which does use auto throttle
    // and navigation
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    return true;
}

void ModeQRTL::update()
{
    plane.mode_qstabilize.update();
}

