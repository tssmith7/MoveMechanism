// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>

#include "DataLogger.h"
#include "subsystems/ExampleIOSim.h"
#include "subsystems/ExampleSubsystem.h"


ExampleSubsystem::ExampleSubsystem() {
  // Create the IO
  if( frc::RobotBase::IsSimulation() ) {
    io = std::unique_ptr<ExampleIO> (new ExampleIOSim());
  }

  frc::SmartDashboard::PutData("Mech2d", &m_mech);
}


void ExampleSubsystem::Periodic() {
  io->UpdateInputs( inputs );

  inputs.ProcessInputs( "Example" );

  m_elevator->SetLength( inputs.elevPosition.value() );
  m_arm->SetAngle( inputs.armPosition - 90_deg );
  m_wrist->SetAngle( inputs.wristPosition - inputs.armPosition );
  m_xbarRoot->SetPosition( 15, 22 + inputs.elevPosition.value() / 2 );
}

frc2::CommandPtr ExampleSubsystem::MoveToAngles( units::degree_t arm, units::degree_t wrist)
{
  return frc2::cmd::Sequence(
        RunOnce( [this, arm, wrist] { 
            GoToArmAngle( arm );
            GoToWristAngle( wrist );
        }),
        frc2::cmd::WaitUntil( [this] { return IsAtArmGoal(); } ).WithTimeout( 3_s )
    ).WithName( "MoveToAngles" );
}

frc2::CommandPtr ExampleSubsystem::MoveToHeight( units::meter_t height)
{
  return frc2::cmd::Sequence(
        RunOnce( [this, height] { 
            GoToHeight( height );
        }),
        frc2::cmd::WaitUntil( [this] { return IsAtHeightGoal(); } ).WithTimeout( 3_s )
    ).WithName( "MoveToHeight" );
}

void ExampleSubsysInputs::ProcessInputs( std::string key ) {
  LOG_UNIT(key,armPosition)
  LOG_UNIT(key,armGoal)
  LOG_UNIT(key,armVelocity)
  LOG_UNIT(key,armAppliedVolts)
  LOG_UNIT(key,armCurrent)

  LOG_UNIT(key,wristPosition)
  LOG_UNIT(key,wristGoal)
  LOG_UNIT(key,wristVelocity)
  LOG_UNIT(key,wristAppliedVolts)
  LOG_UNIT(key,wristCurrent)

  LOG_UNIT(key,elevPosition)
  LOG_UNIT(key,elevGoal)
  LOG_UNIT(key,elevVelocity)
  LOG_UNIT(key,elevAppliedVolts)
  LOG_UNIT(key,elevCurrent)
}
