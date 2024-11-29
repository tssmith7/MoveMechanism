// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose3d.h>

#include "LoggedCommand.h"

class ExampleSubsystem;
class Drive;
class Intake;

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class NoteViz : public frc2::CommandHelper<LoggedCommand, NoteViz>  {
 public:
  
  explicit NoteViz( Drive* drive, ExampleSubsystem* es, Intake* intake );

  void Execute() override;
  void Ending( bool interrupted ) override;
  bool IsFinished() override;

 private:
    Drive* m_drive; 
    ExampleSubsystem* m_es;
    Intake* m_intake;

    std::optional<frc::Pose3d> notePose;
};
