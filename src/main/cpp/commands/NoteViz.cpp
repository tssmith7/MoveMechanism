// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/NoteViz.h"

#include "subsystems/ExampleSubsystem.h"
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"

NoteViz::NoteViz( Drive* drive, ExampleSubsystem* es, Intake* intake )
    : m_drive{drive}, m_es{es}, m_intake{intake}
{
    SetName( "NoteViz" );

  // This command has no requirements
  //  AddRequirements();
}

void NoteViz::Execute() {
    frc::Transform3d elevTransform, armTransform, wristTransform;
    units::radian_t armAngle, wristRelAngle;

    armAngle = m_es->GetArmAngle();
    wristRelAngle = m_es->GetWristAngle() - armAngle;

    elevTransform = { -4.5_in, 0_m, m_es->GetHeight() + 7.5_in, frc::Rotation3d{} };

    frc::Translation2d armPos = { 10_in, 0_in };
    armPos = armPos.RotateBy( armAngle );
    armTransform = { -armPos.X(), 0_in, armPos.Y(), { 0_deg, armAngle, 0_deg } };

    frc::Translation2d wristPos = { 6_in, 0_in };
    wristPos = wristPos.RotateBy( wristRelAngle );
    wristTransform = { -wristPos.X(), 0_in, wristPos.Y(), { 0_deg, wristRelAngle, 0_deg } };

    notePose = m_drive->GetPose() + elevTransform + armTransform + wristTransform;

    DataLogger::Log( "NoteViz/notePose", notePose );
}

void NoteViz::Ending( bool interrupted )
{
    notePose.reset();
    DataLogger::Log( "NoteViz/notePose", notePose );
}


bool NoteViz::IsFinished() {
    return !m_intake->HasNote();
}
