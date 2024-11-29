// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose3d.h>

#include "LoggedCommand.h"


class ShotViz : public frc2::CommandHelper<LoggedCommand, ShotViz>  {
 public:
  
  explicit ShotViz( std::function<frc::Pose3d()> poseFunc );

  void Init() override;
  void Execute() override;
  void Ending( bool interrupted ) override;
  bool IsFinished() override;

 private:
    std::function<frc::Pose3d()> poseFunc;
    std::optional<frc::Pose3d> notePose;
    frc::Translation3d shotUnitVector;
};
