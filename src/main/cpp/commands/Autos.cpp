// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc/Timer.h>
#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"
#include "commands/ShotViz.h"

frc2::CommandPtr autos::ShootNote(Intake *intake, Drive *d )
{
  return frc2::cmd::Parallel(
   intake->EjectNote(),
   ShotViz( [d] { return d->GetPose(); } ).AsProxy()
  ).WithName( "ShootNote" );
}

frc2::CommandPtr autos::StateCommand( ) {
  units::second_t timestart = frc::Timer::GetFPGATimestamp();
  
  return frc2::cmd::Sequence( 
    frc2::cmd::Print( fmt::format( "StateCommand started at {}...\n", timestart) ),
    frc2::cmd::Wait( 1_s ),
    frc2::cmd::RunOnce([timestart] {
      fmt::print( "StateCommand got start time {} and end time of {}\n", timestart, frc::Timer::GetFPGATimestamp() );
    } )
  );
}

frc2::CommandPtr autos::StateCommandSubsys(ExampleSubsystem* subsystem) {
  units::second_t timestart = frc::Timer::GetFPGATimestamp();
  
  return frc2::cmd::Sequence( 
    frc2::cmd::Wait( 1_s ),
    frc2::cmd::RunOnce([timestart] {
      fmt::print( "StateCommand got start time {} and end time of {}\n", timestart, frc::Timer::GetFPGATimestamp() );
    } )
  );
}
