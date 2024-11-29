// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/ExampleSubsysInputs.h"
#include "subsystems/ExampleIO.h"


class ExampleSubsystem : public frc2::SubsystemBase {
public:
    ExampleSubsystem();
    
    void Periodic() override;

    void GoToArmAngle( units::degree_t a) { inputs.armGoal = a; io->SetArmGoal( a );}
    void GoToWristAngle( units::degree_t a) { inputs.wristGoal = a; io->SetWristGoal( a );}
    void GoToHeight( units::inch_t h) { inputs.elevGoal = h; io->SetElevGoal( h );}

    units::degree_t GetArmAngle() { return inputs.armPosition;}
    units::degree_t GetWristAngle() { return inputs.wristPosition;}
    units::inch_t GetHeight() { return inputs.elevPosition;}

    bool IsAtArmGoal() { return units::math::abs( inputs.armGoal - GetArmAngle() ) < 1_deg &&
                                units::math::abs( inputs.wristGoal - GetWristAngle() ) < 1_deg ; }

    bool IsAtHeightGoal() { return units::math::abs( inputs.elevGoal - GetHeight() ) < 0.01_m ; }


    frc2::CommandPtr MoveToAngles( units::degree_t arm, units::degree_t wrist);
    frc2::CommandPtr MoveToHeight( units::meter_t height);

private:
    ExampleSubsysInputs inputs;

    std::unique_ptr<ExampleIO> io;

    frc::Mechanism2d m_mech{40/39.0,40/39.0};
        // the mechanism root node
    frc::MechanismRoot2d* m_root = m_mech.GetRoot("root", 16/39.0, 10/39.0);
    // MechanismLigament2d objects represent each "section"/"stage" of the
    // mechanism, and are based off the root node or another ligament object
    frc::MechanismLigament2d* m_elevator =
        m_root->Append<frc::MechanismLigament2d>("elevator", 6/39.0, 90_deg);
    frc::MechanismLigament2d* m_arm =
        m_elevator->Append<frc::MechanismLigament2d>("arm", 10/39.0, 90_deg, 6, frc::Color8Bit{frc::Color::kGreen});
    frc::MechanismLigament2d* m_wrist =
        m_arm->Append<frc::MechanismLigament2d>(
            "wrist", 6/39.0, 0_deg, 6, frc::Color8Bit{frc::Color::kPurple});

    frc::MechanismRoot2d* m_xbarRoot = m_mech.GetRoot("xbroot", 15/39.0, 22/39.0);
    frc::MechanismLigament2d* m_crossbar =
        m_xbarRoot->Append<frc::MechanismLigament2d>("cross bar", 1/39.0, 90_deg, 6, frc::Color8Bit{frc::Color::kRed});

};
