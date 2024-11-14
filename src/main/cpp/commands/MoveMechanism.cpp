// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveMechanism.h"

MoveMechanism::MoveMechanism( ExampleSubsystem* e, 
                              units::degree_t arm_pos, units::degree_t wrist_pos, units::meter_t height )
 : e{e},  m_arm_target{arm_pos}, m_wrist_target{wrist_pos}, m_height_target{ height } {
  SetName( "MoveMechanism" );
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({e});
}

// Called when the command is initially scheduled.
void MoveMechanism::Init() {

    // We need to safely move the mechanism from it's current position
    // to the target position.
    // The constraints are as follows:
    //
    // If the elevator is to be moved up or down then:
    //    The wrist must be at 90_deg while elevator height is > 4_in
    //    The arm must be at 75_deg  while elevator height is > 4_in
    //
    // As arm is moved from (<75_deg) to (>120_deg) then:
    //    The wrist must be at 35_deg.

    // Check if the elevator needs to move through the crash point.
  if( e->GetHeight() > 4_in && m_height_target < 4_in ) {
    m_elev_going_up = false;
    m_elev_going_down = true;
    m_elev_is_done = false;
  } else if( e->GetHeight() < 4_in && m_height_target > 4_in ) {
    m_elev_going_up = true;
    m_elev_going_down = false;
    m_elev_is_done = false;
  } else {
      // Elevator doesn't need to move far.
    m_elev_going_up = false;
    m_elev_going_down = false;
    m_elev_is_done = true;
  }

    // Check if the arm need to move through the crash point.
  if( e->GetArmAngle() < 80_deg && m_arm_target > 120_deg ) {
    m_arm_going_back = true;
    m_arm_going_forward = false;
    m_wrist_retracted = false;
  } else if( e->GetArmAngle() > 120_deg && m_arm_target < 80_deg ) {
    m_arm_going_back = false;
    m_arm_going_forward = true;
    m_wrist_retracted = false;
  } else {
      // Arm doesn't need to move far.
    m_arm_going_back = false;
    m_arm_going_forward = false;
    m_wrist_retracted = true;
  }

  // fmt::print( "MoveMechanism::Init() -- m_elev_going_up({}), m_elev_going_down({}), m_arm_going_back({}), m_arm_going_forward({})\n", 
  //   m_elev_going_up, m_elev_going_down, m_arm_going_back, m_arm_going_forward );

}

// Called repeatedly when this Command is scheduled to run
void MoveMechanism::Execute() {

  if( m_elev_going_up ) {
      // The elevator needs to go up
    if( m_arm_going_forward ) {
      // The arm is back and the elevator needs to go up
      // start with the arm / wrist move forward.
      if( FlipArmForward() ) {
          e->GoToHeight( m_height_target );
          if( e->IsAtHeightGoal() ) {
            m_arm_going_forward = false;
          }
      }
    } else {
        // Should be safe to move everything at once.
      e->GoToHeight( m_height_target );
      e->GoToArmAngle( m_arm_target );
      e->GoToWristAngle( m_wrist_target );
      m_elev_is_done = true;
    }
  } else if( m_elev_going_down ) {
      // If the elevator needs to come down
    if( e->GetWristAngle() > 95_deg && !m_elev_is_done ) {
      // The wrist is over the cross bar and needs to extend.
      e->GoToWristAngle( 90_deg );
    } else {
      if( m_arm_going_back ) {
        if( !m_elev_is_done ) {
          e->GoToHeight( m_height_target );
          if( e->GetHeight() - 4_in < 0_in ) {
            // We got the elevator low enough.
            m_elev_is_done = true;
          }
        } else {
          FlipArmBackward();
        }
      } else {
          // Should be safe to move everything at once.
        m_elev_is_done = true;
        e->GoToHeight( m_height_target );
        e->GoToArmAngle( m_arm_target );
        e->GoToWristAngle( m_wrist_target );
      }
    }
  } else if( m_arm_going_forward ) {
      // Transitioning from backward to forward with the arm.
    FlipArmForward();
    e->GoToHeight( m_height_target );
  } else if( m_arm_going_back ) {
      // Transitioning from forward to backward with the arm.
    FlipArmBackward();
    e->GoToHeight( m_height_target );
  } else {
      // Should be safe to move everything at once.
    e->GoToHeight( m_height_target );
    e->GoToArmAngle( m_arm_target );
    e->GoToWristAngle( m_wrist_target );
  }
}

// Called once the command ends or is interrupted.
void MoveMechanism::Ending(bool interrupted) {}

// Returns true when the command should end.
bool MoveMechanism::IsFinished() {
  // fmt::print( "Arm({:.5}), Wrist({:.5}), Elev({:.5}), m_elev_is_done({}),m_wrist_retracted({}), ArmAtGoal({}), HeightAtGoal({})\n", 
  //     e->GetArmAngle(), e->GetWristAngle(), e->GetHeight(), m_elev_is_done, m_wrist_retracted, e->IsAtArmGoal(), e->IsAtHeightGoal() );
  return m_elev_is_done && m_wrist_retracted && e->IsAtArmGoal() && e->IsAtHeightGoal();
}

bool MoveMechanism::FlipArmForward() {

  if( !m_wrist_retracted ) {
      // The wrist hasn't retracted yet.
    e->GoToWristAngle( kWristRetractAngle );
    if( units::math::abs( e->GetWristAngle() - kWristRetractAngle ) < 5_deg ) {
      // We got the wrist to the correct place.
      m_wrist_retracted = true;
      e->GoToArmAngle( m_arm_target );
    } 
  } else {
      // The wrist is retracted but the arm still needs to move forward.
    if( e->GetArmAngle() - 100_deg < 0_deg ) {
      // Wait until the arm angle is less than 100_deg to move the wrist
      // and elevator to the target position.
      if( m_wrist_target > 90_deg ) {
        e->GoToWristAngle( 90_deg );
      } else {
        e->GoToWristAngle( m_wrist_target );
      }
    } else {
      // Arm is still coming forward.  We can bring the wrist up
      // If the target wrist position is upward.
      if( m_wrist_target > kWristRetractAngle ) {
        units::degree_t wristCurve = units::math::min(m_wrist_target, 90_deg) - (e->GetArmAngle() - 100_deg);
        if( wristCurve > kWristRetractAngle ) {
          e->GoToWristAngle( wristCurve );
        }
      }
    }
  }

  return m_wrist_retracted && e->IsAtArmGoal();
}

bool MoveMechanism::FlipArmBackward() {

  if( !m_wrist_retracted ) {
    e->GoToArmAngle( m_arm_target );
    e->GoToWristAngle( kWristRetractAngle );
    if( e->GetArmAngle() - e->GetWristAngle() < 20_deg ) {
       e->GoToWristAngle( e->GetArmAngle() - 20_deg );
    }
    if( units::math::abs( e->GetWristAngle() - kWristRetractAngle ) < 5_deg ) {
      // We got the wrist to the correct place.
      m_wrist_retracted = true;
      e->GoToArmAngle( m_arm_target );
    } 
  } else {
      // The wrist is retracted but the arm still needs to move backward.
    if( e->GetArmAngle() - 120_deg > 0_deg ) {
      // Wait until the arm angle is greater than 120_deg to move the wrist
      // to the target position.
      e->GoToWristAngle( m_wrist_target );
    }
  } 

  return m_wrist_retracted && e->IsAtArmGoal();
}
