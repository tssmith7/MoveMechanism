#pragma once

#include <string>
#include <vector>

#include "subsystems/ExampleSubsysInputs.h"

class ExampleIO {
public:
    virtual void UpdateInputs( ExampleSubsysInputs &inputs ) = 0;

    virtual void SetArmGoal( units::degree_t goal ) = 0;
    virtual void SetWristGoal( units::degree_t goal ) = 0;
    virtual void SetElevGoal( units::inch_t goal ) = 0;
};
