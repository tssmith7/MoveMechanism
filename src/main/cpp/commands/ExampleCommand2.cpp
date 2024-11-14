// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExampleCommand2.h"
#include <frc/Timer.h>

ExampleCommand2::ExampleCommand2(ExampleSubsystem* subsystem)
    : m_subsystem{subsystem} {
  SetName( "ExampleCommand2" );
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}

void ExampleCommand2::Init() {
  start = frc::Timer::GetFPGATimestamp();
}

bool ExampleCommand2::IsFinished() {
  return frc::Timer::GetFPGATimestamp() - start > 2_s;
}
