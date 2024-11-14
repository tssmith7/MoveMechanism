// Log data to the Data Log file on the RoboRIO
//

#pragma once

#include <span>
#include <map>

#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/GenericEntry.h>

#include <frc/geometry/Pose2d.h>

class DataLogger {
  private:
      // This class is a singleton.
    static DataLogger *singleton;

      // Constructor is private
    DataLogger() {}
  public:

      // delete copy constructor
    DataLogger(const DataLogger& obj) = delete; 

    static DataLogger& GetInstance() {
            // If there is no instance of class
            // then we can create an instance.
      if (singleton == nullptr)  {
        singleton = new DataLogger();
        singleton->log = &frc::DataLogManager::GetLog();
        singleton->nt_inst = nt::NetworkTableInstance::GetDefault();
      }
            
      return *singleton;
    }

    void Send( std::string_view s, double val );
    void Send( std::string_view s, std::span<const double> a );
    void Send( std::string_view s, std::string_view val );
    void Send( std::string_view s, int val );
    void Send( std::string_view s, bool val );
    void Send( std::string_view s, frc::Pose2d p );

    void SendNT( std::string s, double val );
    void SendNT( std::string s, std::span<const double> a );
    void SendNT( std::string s, frc::Pose2d p );

    void Log(  std::string s );

    void LogMetadata( void );

  private:
    wpi::log::DataLog *log;
    nt::NetworkTableInstance nt_inst;
    std::map<std::string, nt::GenericPublisher> nt_map;

    void SendMetadata( std::string_view s, std::string_view val );

};

class LoggedInputs {
public:
  virtual void ProcessInputs( std::string key ) =0;
};

#define LOG_VARIABLE(key,v) DataLogger::GetInstance().SendNT( key + "/" + #v, v );
#define LOG_UNIT(key,v) DataLogger::GetInstance().SendNT( key + "/" + #v + "(" + units::abbreviation(v) + ")", v.to<double>() );