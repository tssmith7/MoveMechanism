#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>

#include <units/angular_velocity.h>

class Intake : public frc2::SubsystemBase {
public:
    Intake();
    void Periodic();
    bool HasNote() { return inputs.hasNote; }
    frc2::Trigger HasNoteTrigger() {return frc2::Trigger( [this] { return inputs.hasNote; } ); }

    frc2::CommandPtr IntakeNote();
    frc2::CommandPtr EjectNote( bool forward=true );

    struct Inputs {
        bool hasNote;
        units::revolutions_per_minute_t speed;

        void Log( std::string key );
    };

private:
    Inputs inputs;
};
