#ifndef INPUTHANDLING_H
#define INPUTHANDLING_H

#endif // INPUTHANDLING_H

#include "../Common/ControlInputsProtocol.h"

namespace InputHandling
{
    bool GetKeyStateDown(int nVirtKey);

    bool GetMoveInput(ControlInputsProtocol::Message::MoveSpeed& outMoveSpeed, bool& outMoveDirFwd);
    float GetSteerInputNorm();
    bool GetBrakeInput();
}
