// Log data to the Data Log file on the RoboRIO
//

#pragma once

#include <span>
#include <map>
#include <units/base.h>
#include <units/math.h>
#include <wpi/array.h>

#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/GenericEntry.h>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModuleState.h>

#define AUTOLOG(key,v) DataLogger::Log( key + "/" + #v, v );

class DataLogger {
private:
    // This class is a singleton.
    static DataLogger *singleton;

    // Constructor is private
    DataLogger() {}
    static DataLogger& GetInstance();

public:

    // delete copy constructor
    DataLogger(const DataLogger& obj) = delete; 

        // Base Log() functions that other Log() functions call.
    static void Log( const std::string& s, double val, bool alsoNT=false );
    static void Log( const std::string& s, std::span<const double> a, bool alsoNT=false );
    static void Log( const std::string& s, int val, bool alsoNT=false );
    static void Log( const std::string& s, int64_t val, bool alsoNT=false );
    static void Log( const std::string& s, std::span<const int64_t> a, bool alsoNT=false );
    static void Log( const std::string& s, bool val, bool alsoNT=false );
    static void Log( const std::string& s, std::span<const bool> a, bool alsoNT=false );
    static void Log( const std::string& s, const std::string& val, bool alsoNT=false );

        // Derived type Log() functions
        // These call the Base Log() functions above
    static void Log( const std::string& s, const frc::Pose2d& p, bool alsoNT=false );
    static void Log( const std::string &s, const wpi::array<frc::SwerveModuleState, 4U> &sms, bool alsoNT=false );

        // A units library type.
    template <class UnitType>
    requires units::traits::is_unit_t<UnitType>::value
    static void Log( const std::string &s, const UnitType& val, bool alsoNT=false ) noexcept;

        // A vector of units library type.
    template <class UnitType>
    requires units::traits::is_unit_t<UnitType>::value
    static void Log( const std::string &s, const std::vector<UnitType>& vec, bool alsoNT=false ) noexcept;

        // A vector. Must be specialized
    template<class T>
    static void Log( const std::string &s, const std::vector<T>& vec, bool alsoNT=false );

    static void Log( const std::string& s );

    static void LogMetadata( void );

private:
    void Send( const std::string& s, double val );
    void Send( const std::string& s, std::span<const double> a );
    void Send( const std::string& s, int64_t val );
    void Send( const std::string& s, std::span<const int64_t> a );
    void Send( const std::string& s, bool val );
    void Send( const std::string& s, std::span<const bool> a );
    void Send( const std::string& s, const std::string& val );

    void SendNT( const std::string& s, double val );
    void SendNT( const std::string& s, std::span<const double> a );
    void SendNT( const std::string& s, int64_t val );
    void SendNT( const std::string& s, std::span<const int64_t> a );
    void SendNT( const std::string& s, bool val );
    void SendNT( const std::string& s, std::span<const bool> a );
    void SendNT( const std::string& s, const std::string& val );

    wpi::log::DataLog *log;
    std::shared_ptr<nt::NetworkTable> nt_table;
    std::map<std::string, nt::GenericPublisher> nt_map;
    bool isFMSAttached;

    void SendMetadata( std::string_view s, std::string_view val );
};

class LoggedInputs {
public:
    virtual void ProcessInputs( std::string key ) =0;
};

template <class UnitType>
requires units::traits::is_unit_t<UnitType>::value
void DataLogger::Log( const std::string &s, const UnitType& val, bool alsoNT ) noexcept {
    DataLogger::Log( s + "(" + units::abbreviation(val) + ")", val.to<double>(), alsoNT );
}

template <class UnitType>
requires units::traits::is_unit_t<UnitType>::value
void  DataLogger::Log( const std::string &s, const std::vector<UnitType>& vec, bool alsoNT ) noexcept {
    static std::vector<double> a{256};
    a.clear();
    for( size_t i=0; i<vec.size(); ++i ) {
        a.push_back(vec[i].value());
    }
    DataLogger::Log( s + "(" + units::abbreviation(vec[0]) + ")", std::span<const double>( a ), alsoNT );
}