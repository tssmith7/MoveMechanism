// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <vector>
#include <optional>

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>

#include <frc/geometry/Pose2d.h>

class ExampleSubsysInputs {
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

    std::vector<units::degree_t> armPositionHistory = {0_deg, 0_deg};
    std::vector<double> armPositionHistoryDbl = {0, 0};
    std::optional<units::degree_t> armSetpoint;
    std::optional<frc::Pose2d> visionPose;
    std::vector<int64_t> int64vec = { 1, 2, 3, 4 };
    bool has10selasped = false;
    std::array<bool,10> oneSecInc = {};
};

