#include "mode.h"
#include "Plane.h"

bool ModeQRTL::_enter()
{
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    return plane.mode_qstabilize._enter();
}

void ModeQRTL::update()
{
    plane.mode_qstabilize.update();
}

