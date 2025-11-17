#include "InputHandling.h"

#include "windows.h"

namespace InputHandling
{

using TMoveSpeed = ControlInputsProtocol::Message::MoveSpeed;

bool GetKeyStateDown(int nVirtKey)
{
    SHORT state = GetKeyState(nVirtKey);
    return (state & (~1)) != 0;
}

bool GetMoveInput(TMoveSpeed& outMoveSpeed, bool& outMoveDirFwd)
{
    bool move = false;
    bool moveDirFwd = false;
    TMoveSpeed moveSpeed = TMoveSpeed::SLOW;

    bool inputFwd = GetKeyStateDown('w') != 0 || GetKeyStateDown('W') != 0;
    bool inputBwd = GetKeyStateDown('s') != 0 || GetKeyStateDown('S') != 0;

    if (inputFwd && inputBwd)
    {
        move = false;
        moveDirFwd = false;
    }
    else if (inputFwd)
    {
        move = true;
        moveDirFwd = true;
    }
    else if (inputBwd)
    {
        move = true;
        moveDirFwd = false;
    }

    if (move)
    {
        // Resolve move speed
        if (GetKeyStateDown(VK_SHIFT))
            moveSpeed = TMoveSpeed::FAST;
        else if (GetKeyStateDown(VK_CONTROL))
            moveSpeed = TMoveSpeed::SLOW;
        else
            moveSpeed = TMoveSpeed::CRUISE;
    }

    outMoveSpeed = moveSpeed;
    outMoveDirFwd = moveDirFwd;
    return move;
}

float GetSteerInputNorm()
{
    bool inputLeft = GetKeyStateDown('a') != 0 || GetKeyStateDown('A') != 0;
    bool inputRight = GetKeyStateDown('d') != 0 || GetKeyStateDown('D') != 0;

    if (inputLeft && inputRight)
        return 0.0f;
    else if (inputRight)
        return 1.0f;
    else if (inputLeft)
        return -1.0f;

    return 0.0f;
}

bool GetBrakeInput()
{
    // Holding both W and S
    if ((GetKeyStateDown('w') || GetKeyStateDown('W')) &&
        (GetKeyStateDown('s') || GetKeyStateDown('S')))
        return true;

    if (GetKeyStateDown('x') || GetKeyStateDown('X'))
        return true;

    if (GetKeyStateDown(VK_SPACE))
        return true;

    return false;
}
}
