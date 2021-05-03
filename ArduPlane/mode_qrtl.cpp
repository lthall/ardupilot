#include "mode.h"
#include "Plane.h"

bool ModeQRTL::_enter()
{
    plane.mode_qstabilize._enter();
    return true;
}

void ModeQRTL::update()
{
    plane.mode_qstabilize.update();
}

