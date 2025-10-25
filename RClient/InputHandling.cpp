#include "InputHandling.h"

#include "windows.h"

namespace InputHandling
{
bool GetKeyStateDown(int nVirtKey)
{
    SHORT state = GetKeyState(nVirtKey);
    return (state & (~1)) != 0;
}

float GetMotorInputNorm()
{
    if (GetKeyStateDown('w') != 0 || GetKeyStateDown('W') != 0)
        return 1.0f;
    else if (GetKeyStateDown('s') != 0 || GetKeyStateDown('S') != 0)
        return -1.0f;

    return 0.0f;
}

float GetSteerInputNorm()
{
    if (GetKeyStateDown('d') != 0 || GetKeyStateDown('D') != 0)
        return 1.0f;
    else if (GetKeyStateDown('a') != 0 || GetKeyStateDown('A') != 0)
        return -1.0f;

    return 0.0f;
}

bool GetBrakeInput()
{
    // Holding both W and S
    if ((GetKeyStateDown('w') != 0 || GetKeyStateDown('W') != 0) &&
        (GetKeyStateDown('s') != 0 || GetKeyStateDown('S') != 0))
        return true;

    return (GetKeyStateDown(VK_SPACE) != 0);
}
}
