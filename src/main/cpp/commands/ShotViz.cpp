// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShotViz.h"

#include "subsystems/ExampleSubsystem.h"
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"

ShotViz::ShotViz( std::function<frc::Pose3d()> poseFunc ) : poseFunc{poseFunc}
{
    SetName( "ShotViz" );

  // This command has no requirements
  //  AddRequirements();
}

void ShotViz::Init()
{
    // Get the robot pose when shot.
    frc::Pose3d robotPose = poseFunc();
    units::degree_t shooterAngle = 35_deg;

    frc::Transform3d shooterTrans = { 8_in, 0_in, 12_in, { 0_deg, -shooterAngle, 0_deg } };

    notePose = robotPose + shooterTrans;

    DataLogger::Log( "NoteViz/notePose", notePose );
}

void ShotViz::Execute() {
    frc::Transform3d delta;
    units::meter_t deltaMeters = 5_mps * 20_ms;

    delta = { deltaMeters, 0_m, 0_m, frc::Rotation3d{} };

    notePose = notePose.value() + delta;

    DataLogger::Log( "NoteViz/delta", delta );
    DataLogger::Log( "NoteViz/notePose", notePose );
}

void ShotViz::Ending( bool interrupted )
{
    notePose.reset();
    DataLogger::Log( "NoteViz/notePose", notePose );
}


bool ShotViz::IsFinished() {
    if( !notePose ) {
        return true;
    }

    return notePose.value().X() < 0_ft || notePose.value().X() > 55_ft || notePose.value().Y() < 0_ft || notePose.value().Y() > 30_ft;
}
