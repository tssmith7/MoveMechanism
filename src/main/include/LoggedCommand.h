// Log data to the Data Log file on the RoboRIO
//

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "DataLogger.h"

class LoggedCommand : public frc2::Command {
public:
  

  void Initialize() final {
    // DataLogger::GetInstance().Send( "Command/" + this->GetName(), true );
    fmt::print( "   Command {} initialized\n", this->GetName() );
    Init();
  }
  void End( bool interrupted ) final {
    Ending( interrupted );
    // DataLogger::GetInstance().Send( "Command/" + this->GetName(), false );
    fmt::print( "   Command {} ended.\n", this->GetName() );
  }

  virtual void Init() {}
  virtual void Ending( bool interrupted ) {}
};
