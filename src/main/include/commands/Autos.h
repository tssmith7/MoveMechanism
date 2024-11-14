// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "LoggedCommand.h"
#include "subsystems/ExampleSubsystem.h"

namespace autos {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr ExampleAuto(ExampleSubsystem* subsystem);

frc2::CommandPtr StateCommand();
frc2::CommandPtr StateCommandSubsys(ExampleSubsystem* subsystem);

}  // namespace autos

class StatefulCommand : public frc2::CommandHelper<frc2::SequentialCommandGroup, StatefulCommand> {
public:
    StatefulCommand( ExampleSubsystem *es ) { 
        units::second_t timestart = frc::Timer::GetFPGATimestamp();

        AddCommands( 
            frc2::WaitCommand( 1_s ),
            frc2::InstantCommand([timestart] {
            fmt::print( "StateCommand got start time {} and end time of {}\n", timestart, frc::Timer::GetFPGATimestamp() );
            }, {es} )        );
    }

};

class Stateful {
public:
    Stateful() {}
    
    void Schedule() { 
        cmd = CommandFactory().Unwrap();
        cmd->Schedule();
    }
    
private:
    virtual frc2::CommandPtr CommandFactory() = 0;

    std::unique_ptr<frc2::Command> cmd;
};


class TimerCommand : public Stateful {
public:
    TimerCommand( ExampleSubsystem *es ) : es(es) {}
    
private:
    frc2::CommandPtr CommandFactory() {
        units::second_t timestart = frc::Timer::GetFPGATimestamp();

        return autos::StateCommandSubsys(es);
    }

    ExampleSubsystem *es;
};

class DelayedStop {
public:
    void setStart() { 
        fmt::print( "DelayStop::setStart - called\n" );
        if( !cmd_running ) { 
            m_start = frc::Timer::GetFPGATimestamp(); 
            fmt::print( "DelayStop::setStart - starting time {}\n", m_start );
        }
    }

    bool AtEnd() {
        bool atend = frc::Timer::GetFPGATimestamp() - m_start >= 2_s;
        fmt::print( "DelayStop::AtEnd - start {}, now {}, at end? = {}\n", m_start, frc::Timer::GetFPGATimestamp(), atend );
        if( atend ) cmd_running = false;
        return atend;
    }

    frc2::CommandPtr DelayCommand() {
        return frc2::cmd::RunOnce( [this] { setStart(); } ).AndThen( frc2::cmd::WaitUntil( [this] { return AtEnd(); }));
    }

    frc2::CommandPtr DelayCommand2() {
        return frc2::cmd::Sequence(
            frc2::cmd::RunOnce( [this] { setStart(); } ),
            frc2::cmd::WaitUntil( [this] { return AtEnd(); }).WithTimeout( 1_s )
        );
    }

private:
    bool cmd_running{false};
    units::second_t m_start;
};