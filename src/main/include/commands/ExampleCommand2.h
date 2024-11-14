// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "LoggedCommand.h"
#include "subsystems/ExampleSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ExampleCommand2
    : public frc2::CommandHelper<LoggedCommand, ExampleCommand2> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit ExampleCommand2(ExampleSubsystem* subsystem);

  void Init();
  bool IsFinished();

 private:
  ExampleSubsystem* m_subsystem;

  units::second_t start;
};
