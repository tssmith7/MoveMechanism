// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include <frc/DriverStation.h>

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>

#include "Robot.h"
#include "DataLogger.h"

std::string PoseToString( const frc::Pose2d &p );
std::string TransformToString( const frc::Transform2d &t );

void RotateToTarget( const frc::Pose2d &target, frc::Pose2d robot );

Robot::Robot() {

}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  LoggedRobot::RobotPeriodic();
  frc2::CommandScheduler::GetInstance().Run();  
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // fmt::print( "\n\nBLUE Negative Phi, Negative Theta\n");
  // RotateToTarget( blueTarget, {2_m, 1.5_m, -30_deg} );

  // fmt::print( "\n\nBLUE Positive Phi, Postitive Theta\n");
  // RotateToTarget( blueTarget, {2_m, 2.8_m, 30_deg} );

  // fmt::print( "\n\nBLUE Positive Phi, Postitive Theta\n");
  // RotateToTarget( blueTarget, {2_m, 2.8_m, -30_deg} );

  // fmt::print( "\n\nBLUE Positive Phi, Postitive Theta Facing Wrong Way\n");
  // RotateToTarget( blueTarget, {2_m, 1.5_m, -130_deg} );

  // fmt::print( "\n\nBLUE Positive Phi, Postitive Theta\n");
  // RotateToTarget( blueTarget, {2_m, 2.8_m, 175_deg} );

  // fmt::print( "\n\nRED Negative Phi, Postitive Theta\n");
  // RotateToTarget( redTarget, {8_m, 1.5_m, 160_deg} );

  // fmt::print( "\n\nRED Negative Phi, Negative Theta\n");
  // RotateToTarget( redTarget, {8_m, 1.5_m, -160_deg} );

  // fmt::print( "\n\nRED Positive Phi, Postitive Theta\n");
  // RotateToTarget( redTarget, {8_m, 2.8_m, 160_deg} );

  // fmt::print( "\n\nRED Positive Phi, Postitive Theta\n");
  // RotateToTarget( redTarget, {8_m, 2.8_m, -160_deg} );
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
    // RotateToTarget( );
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

std::string PoseToString( const frc::Pose2d &p )
{
    return fmt::format( "<{:.5}, {:.5}, {:.5}>", p.X(), p.Y(), p.Rotation().Degrees() );
}

std::string TransformToString( const frc::Transform2d &t )
{
    return fmt::format( "<{:.5}, {:.5}, {:.5}>", t.X(), t.Y(), t.Rotation().Degrees() );
}

// void Robot::RotateToTarget(  )
// {


//     units::meter_t delta_x = robotPose.X() - target.X();
//     units::meter_t delta_y = robotPose.Y() - target.Y();

//     units::degree_t planeAngle = units::math::atan2( delta_y, delta_x );

//   if( units::math::abs( robotPose.Rotation().Degrees() - planeAngle) < 1_deg ) {
//       return;
//   }
//     units::meter_t dist_to_target = (robotPose - target).Translation().Norm(); 

//     units::degree_t spin = planeAngle - robotPose.Rotation().Degrees();
//     units::degree_t corr_spin = spin;
//     if( spin > 180_deg ) {
//       corr_spin = spin - 360_deg;
//     } else if( spin < -180_deg ) {
//       corr_spin = spin + 360_deg;
//     }
//     double correction = corr_spin.value() * 0.05;

//     // fmt::print( "target {}, robot {}, Delta ({:.5},{:.5}), planeAngle {:.5}, spin {:.5}, corr_spin {:.5}, dist{}\n", 
//     // PoseToString(target), PoseToString(robotPose), delta_x, delta_y, planeAngle, spin, corr_spin, dist_to_target);

//     robotPose = robotPose + frc::Transform2d{ 0_m, 0_m, ( correction * 1_deg ) };

// }
