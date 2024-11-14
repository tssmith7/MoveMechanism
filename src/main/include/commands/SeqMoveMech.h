// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "LoggedCommand.h"
#include "Constraint.h"

class ExampleSubsystem;

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SeqMoveMech
    : public frc2::CommandHelper<LoggedCommand, SeqMoveMech> {
 public:
  using Point = Constraint<units::degrees, units::degrees>::Point;

  SeqMoveMech( ExampleSubsystem* e, units::degree_t arm_pos, units::degree_t wrist_pos );

  void Init() override;

  void Execute() override;

  bool IsFinished() override;

 private:
  void NewPathSegment( );

  ExampleSubsystem *e;
  units::degree_t m_arm_target;
  units::degree_t m_wrist_target;
  Constraint<units::degrees, units::degrees> arm_wrist_Cnst;
  std::vector<Point> path;

  Point m_startpt, m_endpt; 
  size_t current_path;
  double t_parameter;
  double step_size;
};
