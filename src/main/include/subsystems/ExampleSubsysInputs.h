// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>

#include "DataLogger.h"


class ExampleSubsysInputs : public LoggedInputs {
public :
    void ProcessInputs( std::string key );

    units::degree_t armPosition = 0_deg;
    units::degree_t armGoal = 0_deg;
    units::revolutions_per_minute_t armVelocity = 0_rpm;
    units::volt_t armAppliedVolts = 0_V;
    units::ampere_t armCurrent = 0_A;

    units::degree_t wristPosition = 0_deg;
    units::degree_t wristGoal = 0_deg;
    units::revolutions_per_minute_t wristVelocity = 0_rpm;
    units::volt_t wristAppliedVolts = 0_V;
    units::ampere_t wristCurrent = 0_A;

    units::inch_t elevPosition = 0_in;
    units::inch_t elevGoal = 0_in;
    units::meters_per_second_t elevVelocity = 0_mps;
    units::volt_t elevAppliedVolts = 0_V;
    units::ampere_t elevCurrent = 0_A;
};

