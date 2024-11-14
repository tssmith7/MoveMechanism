// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExampleCommand.h"
#include <frc/Timer.h>

ExampleCommand::ExampleCommand(ExampleSubsystem* subsystem)
    : m_subsystem{subsystem} {
  SetName( "ExampleCommand" );
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}

void ExampleCommand::Init() {
  start = frc::Timer::GetFPGATimestamp();
  fmt::print( "ExampleCommand::Init\n");
}

bool ExampleCommand::IsFinished() {
  return frc::Timer::GetFPGATimestamp() - start > 1_s;
}
