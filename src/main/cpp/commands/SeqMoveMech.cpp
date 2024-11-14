// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SeqMoveMech.h"

#include "subsystems/ExampleSubsystem.h"

SeqMoveMech::SeqMoveMech( ExampleSubsystem* e, 
                          units::degree_t arm_pos, units::degree_t wrist_pos )
 : e{e},  m_arm_target{arm_pos}, m_wrist_target{wrist_pos} {
  SetName( "SeqMoveMech" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({e});

  arm_wrist_Cnst.AddConstraint( {70_deg, 60_deg }, true );
  arm_wrist_Cnst.AddConstraint( {130_deg, 40_deg }, true );

}

// Called when the command is initially scheduled.
void SeqMoveMech::Init() {

  path = arm_wrist_Cnst.FindPath( {e->GetArmAngle(), e->GetWristAngle()}, {m_arm_target, m_wrist_target} );

  current_path = 0;
  NewPathSegment();
}

// Called repeatedly when this Command is scheduled to run
void SeqMoveMech::Execute() {
  Point current_pt;

  current_pt.x = m_endpt.x * t_parameter + m_startpt.x * ( 1.0 - t_parameter );
  current_pt.y = m_endpt.y * t_parameter + m_startpt.y * ( 1.0 - t_parameter );
  e->GoToArmAngle( current_pt.x );
  e->GoToWristAngle( current_pt.y );

  if( units::math::abs( e->GetArmAngle() - current_pt.x ) < 3_deg &&
      units::math::abs( e->GetWristAngle() - current_pt.y ) < 3_deg ) {
    t_parameter += step_size;
    if( t_parameter >= 1.0 ) {
      current_path++;
      NewPathSegment();
    }
  }
}

// Returns true when the command should end.
bool SeqMoveMech::IsFinished() {
   return current_path == path.size()-1;
}

void SeqMoveMech::NewPathSegment( )
{
  if( current_path == path.size()-1 ) {
    return;
  }

  t_parameter = 0.0;
  m_startpt = path[current_path];
  m_endpt = path[current_path+1];

    // Find the largest degree change along the path
  units::degree_t max_delta = units::math::max( units::math::abs( m_endpt.x - m_startpt.x ), 
                                                units::math::abs( m_endpt.y - m_startpt.y ) );

    // Make the step size such that each step (20_ms) is 0.5 degrees.
  step_size = 0.5_deg / max_delta;
}
