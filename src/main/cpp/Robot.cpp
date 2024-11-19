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

#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

#include "Robot.h"
#include "DataLogger.h"

std::string PoseToString( const frc::Pose2d &p );
std::string TransformToString( const frc::Transform2d &t );

void RotateToTarget( const frc::Pose2d &target, frc::Pose2d robot );

Robot::Robot() {
  const frc::Pose2d redTarget{652.7_in, 218.4_in, 180_deg};
  const frc::Pose2d blueTarget{-1.5_in, 218.4_in, 0_deg};

  target = redTarget;
  robotPose = {14_m, 4.5_m, 170_deg};
  DataLogger::Log( "Robot/Pose", robotPose );


  frc::SmartDashboard::PutData( &field );


  arm_wrist_Cnst.AddConstraint( {70_deg, 35_deg }, true );
  arm_wrist_Cnst.AddConstraint( {110_deg, 40_deg }, true );

  arm_wrist_Cnst.FindPath( {135_deg, 135_deg}, {30_deg, 10_deg} );
  arm_wrist_Cnst.FindPath(  {30_deg, 10_deg}, {135_deg, 135_deg} );

  frc::TrapezoidProfile<units::meter> distProfile{ {1_mps, 4_mps_sq} };
  frc::TrapezoidProfile<units::degree> angProfile{ {540_deg_per_s, 1500_deg_per_s_sq} };

  for( int i=0; i<10; ++i ) {
    units::degree_t movang = 30_deg + 20_deg * i;
    units::meter_t movdist = 0.1_m + 0.1_m * i;
    distProfile.Calculate( 20_ms, {0_m, 0_mps}, {movdist, 0_mps} );
    units::second_t distTime = distProfile.TotalTime();
    angProfile.Calculate( 20_ms, {0_deg, 0_deg_per_s}, {movang, 0_deg_per_s} );
    units::second_t angTime = angProfile.TotalTime();

    fmt::print( "Default dist {} => time {}, ang {} => time {} \n", movdist, distTime, movang, angTime );

    // if( distTime > angTime ) {
    //   TrapezoidProfileByTime<units::degree> slowAngle{ distTime, movang, 1080_deg_per_s_sq };
    //   slowAngle.Calculate( 20_ms, {0_deg, 0_deg_per_s} );
    //   angTime = slowAngle.TotalTime();
    // } else {
    //   TrapezoidProfileByTime<units::meter> slowDist{ angTime, movdist, 4_mps_sq };
    //   slowDist.Calculate( 20_ms, {0_m, 0_mps} );
    //   distTime = slowDist.TotalTime();
    // }
    units::second_t longestTime = units::math::max( distTime, angTime );
    TrapezoidProfileByTime<units::degree> syncAngle{ longestTime, movang, {540_deg_per_s, 1500_deg_per_s_sq} };
    TrapezoidProfileByTime<units::meter> syncDist{ longestTime, movdist, {1_mps, 4_mps_sq} };
    syncAngle.Calculate( 20_ms, {0_deg, 0_deg_per_s} );
    syncDist.Calculate( 20_ms, {0_m, 0_mps} );
    fmt::print( "   Synchronized dist time {}, ang time {}\n", syncAngle.TotalTime(), syncDist.TotalTime() );
  }

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
  static int count = 0;

  frc2::CommandScheduler::GetInstance().Run();

  field.SetRobotPose( robotPose );
  
  ++count;
  
  DataLogger::Log( "Count", count );

  if( count < 200 ) {
    DataLogger::Log( "Robot/Pose", robotPose );
  } else {
    if( count % 300 == 0 ) {
      DataLogger::Log( "Robot/Pose", std::span<double>{} );
    } else if( count % 50 == 0 ) {
      DataLogger::Log( "Robot/Pose", robotPose );
    }

    if( count % 50 == 0 ) {
      fmt::print( "FMS Attached = {}\n", frc::DriverStation::IsFMSAttached() );
    }
  }
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
    RotateToTarget( );
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

void Robot::RotateToTarget(  )
{


    units::meter_t delta_x = robotPose.X() - target.X();
    units::meter_t delta_y = robotPose.Y() - target.Y();

    units::degree_t planeAngle = units::math::atan2( delta_y, delta_x );

  if( units::math::abs( robotPose.Rotation().Degrees() - planeAngle) < 1_deg ) {
      return;
  }
    units::meter_t dist_to_target = (robotPose - target).Translation().Norm(); 

    units::degree_t spin = planeAngle - robotPose.Rotation().Degrees();
    units::degree_t corr_spin = spin;
    if( spin > 180_deg ) {
      corr_spin = spin - 360_deg;
    } else if( spin < -180_deg ) {
      corr_spin = spin + 360_deg;
    }
    double correction = corr_spin.value() * 0.05;

    // fmt::print( "target {}, robot {}, Delta ({:.5},{:.5}), planeAngle {:.5}, spin {:.5}, corr_spin {:.5}, dist{}\n", 
    // PoseToString(target), PoseToString(robotPose), delta_x, delta_y, planeAngle, spin, corr_spin, dist_to_target);

    robotPose = robotPose + frc::Transform2d{ 0_m, 0_m, ( correction * 1_deg ) };

}
