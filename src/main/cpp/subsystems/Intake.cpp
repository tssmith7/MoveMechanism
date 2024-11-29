
#include <frc2/command/Commands.h>
#include "DataLogger.h"

#include "subsystems/Intake.h"


Intake::Intake()
{

}

void Intake::Periodic()
{
    inputs.Log( "Intake" );
}

void Intake::Inputs::Log( std::string key )
{
    AUTOLOG( key, speed );
    AUTOLOG( key, hasNote );
}

frc2::CommandPtr Intake::IntakeNote()
{
    return frc2::cmd::Run( [this] { inputs.speed = 4000_rpm; inputs.hasNote=false; }, {this} )
        .WithTimeout( 0.5_s )
        .AndThen( frc2::cmd::RunOnce( [this] { inputs.speed = 0_rpm; inputs.hasNote = true; }, {this} ) )
        .WithName( "IntakeNote" );
}

frc2::CommandPtr Intake::EjectNote( bool forward )
{
    return frc2::cmd::Sequence(
        frc2::cmd::RunOnce( [this, forward] { 
            units::revolutions_per_minute_t spinSpeed = forward ? -4000_rpm : 4000_rpm;
            inputs.speed = spinSpeed; inputs.hasNote=false; }, {this} ),
        frc2::cmd::Wait( 0.5_s ),
        frc2::cmd::RunOnce( [this] { inputs.speed = 0_rpm; } )
    ).WithName( "EjectNote" );
}
