// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <map>
#include <string>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc2/command/button/CommandJoystick.h>

#include "Constants.h"
#include "commands/Autos.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
  frc2::CommandPtr GetCommand();

 private:
  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
  Drive m_drive;
  Intake m_intake;

  frc2::CommandXboxController controller{0};

  void ConfigureBindings();


  frc::SendableChooser<std::string> m_chooser;

  const std::string kOnePiece = "One Piece";

  const std::string kSourceFourPiece = "Source Four Piece";
  const std::string kSourceThreePiece = "Source Three Piece";
  const std::string kSourceTwoPiece = "Source Two Piece";
  const std::string kSourceTwoPieceCenter = "Source Two Piece Center";
  const std::string kSourceThreePieceCenter = "Source Three Piece Center";
  const std::string kSourceFourPieceCenter = "Source Four Piece Center";
  const std::string kSourceOnePieceTaxi = "Source One Piece Taxi";

  const std::string kAmpFourPiece = "Amp Four Piece";
  const std::string kAmpThreePiece = "Amp Three Piece";
  const std::string kAmpTwoPiece = "Amp Two Piece";
  const std::string kAmpThreePieceCenter = "Amp Three Piece Center";
  const std::string kAmpFourPieceCenter = "Amp Four Piece Center";

  const std::string kMiddleFourPiece = "Middle Four Piece";
  const std::string kMiddleThreePieceAmp = "Middle Three Piece Amp";
  const std::string kMiddleThreePieceSource = "Middle Three Piece Source";
  const std::string kMiddleTwoPiece = "Middle Two Piece";
  const std::string kMiddleFourPieceCenterAmp = "Middle Four Piece Center Amp";
  const std::string kMiddleFourPieceCenterSource = "Middle Four Piece Center Source";

};
