// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/DeferredCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "commands/Autos.h"
#include "commands/MoveMechanism.h"
#include "commands/NoteViz.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();

  m_chooser.SetDefaultOption(kOnePiece, "THEOnePiece");
  m_chooser.AddOption(kSourceFourPiece, "SourceFourPiece");
  m_chooser.AddOption(kSourceThreePiece, "SourceThreePiece");
  m_chooser.AddOption(kSourceTwoPiece, "SourceTwoPiece");
  m_chooser.AddOption(kSourceTwoPieceCenter, "SourceTwoPieceCenter");
  m_chooser.AddOption(kSourceThreePieceCenter, "SourceThreePieceCenter");
  m_chooser.AddOption(kSourceFourPieceCenter, "SourceFourPieceCenter");
  m_chooser.AddOption(kSourceOnePieceTaxi, "SourceOnePieceTaxi");
  m_chooser.AddOption(kAmpFourPiece, "AmpFourPiece");
  m_chooser.AddOption(kAmpThreePiece, "AmpThreePiece");
  m_chooser.AddOption(kAmpTwoPiece, "AmpTwoPiece");
  m_chooser.AddOption(kAmpThreePieceCenter, "AmpThreePieceCenter");
  m_chooser.AddOption(kAmpFourPieceCenter, "AmpFourPieceCenter");
  m_chooser.AddOption(kMiddleFourPiece, "MiddleFourPiece");
  m_chooser.AddOption(kMiddleThreePieceAmp, "MiddleThreePieceAmp");
  m_chooser.AddOption(kMiddleThreePieceSource, "MiddleThreePieceSource");
  m_chooser.AddOption(kMiddleTwoPiece, "MiddleTwoPiece");
  m_chooser.AddOption(kMiddleFourPieceCenterAmp, "MiddleFourPieceCenterAmp");
  m_chooser.AddOption(kMiddleFourPieceCenterSource, "MiddleFourPieceCenterSource");

  frc::SmartDashboard::PutData("Auto Mode", &m_chooser);
 }

void RobotContainer::ConfigureBindings() {
  m_drive.SetDefaultCommand( 
    m_drive.ArcadeDriveCmd( 
      [this] { return controller.GetLeftX(); },
      [this] { return controller.GetLeftY(); },
      [this] { return controller.GetRightX(); }
    )
  );

  m_intake.HasNoteTrigger().OnTrue( 
      frc2::cmd::Sequence(
        frc2::cmd::RunOnce( [this] { controller.SetRumble( frc::GenericHID::kBothRumble, 0.9 ); } ),
        frc2::cmd::Wait( 0.5_s ),
        frc2::cmd::RunOnce( [this] { controller.SetRumble( frc::GenericHID::kBothRumble, 0 ); } )
      )
  );

  m_intake.HasNoteTrigger().OnTrue( NoteViz( &m_drive, &m_subsystem, &m_intake ).ToPtr() );

  controller.A().OnTrue( 
    frc2::cmd::Sequence( 
      MoveMechanism( &m_subsystem, -15_deg, -42_deg, 0_in ).ToPtr(),
      m_intake.IntakeNote(),
      MoveMechanism( &m_subsystem, 155_deg, 135_deg, 0_in ).ToPtr()
    )
  );

  controller.Y().OnTrue( MoveMechanism( &m_subsystem, 75_deg, 120_deg, 20_in ).ToPtr() );
  controller.B().OnTrue( MoveMechanism( &m_subsystem, 155_deg, 40_deg, 0_in ).ToPtr() );
  controller.X().OnTrue( MoveMechanism( &m_subsystem, 155_deg, 135_deg, 0_in ).ToPtr() );

  (controller.RightTrigger() && m_intake.HasNoteTrigger()).OnTrue( autos::ShootNote( &m_intake, &m_drive ) );

  // joystick.Button(1).OnTrue( SeqMoveMech( &m_subsystem, -15_deg, -42_deg ).ToPtr() );
  // joystick.Button(3).OnTrue( SeqMoveMech( &m_subsystem, 155_deg, 40_deg ).ToPtr() );
  // joystick.Button(4).OnTrue( SeqMoveMech( &m_subsystem, 155_deg, 135_deg ).ToPtr() );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // return autos::ExampleAuto(&m_subsystem);

  return pathplanner::AutoBuilder::buildAuto( m_chooser.GetSelected() );
}


frc2::CommandPtr RobotContainer::GetCommand() { 
    units::second_t timestart = frc::Timer::GetFPGATimestamp();

    return frc2::cmd::Sequence ( 
        frc2::WaitCommand( 1_s ).ToPtr(),
        frc2::InstantCommand([timestart] {
        fmt::print( "StateCommand got start time {} and end time of {}\n", timestart, frc::Timer::GetFPGATimestamp() );
        }, {&m_subsystem} ).ToPtr()
      );}
