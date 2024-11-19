// Log data to the Data Log file on the RoboRIO
//

#include <fstream>

#include <wpi/DataLog.h>
#include <frc/Filesystem.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/BooleanArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/GenericEntry.h>
#include "frc/trajectory/Trajectory.h"

#include "DataLogger.h"

#define SEND_LOG_DATA(s,val,alsoNT) \
    if( GetInstance().isFMSAttached ) { \
        GetInstance().Send(s,val); \
        if( alsoNT ) { \
            GetInstance().SendNT(s,val); \
        } \
    } else { \
        GetInstance().SendNT(s,val); \
    } 


DataLogger* DataLogger::singleton = nullptr;

DataLogger& DataLogger::GetInstance() {
    // If there is no instance of class
    // then we can create an instance.
    if (singleton == nullptr)  {
        singleton = new DataLogger();
        singleton->log = &frc::DataLogManager::GetLog();
        singleton->nt_table = nt::NetworkTableInstance::GetDefault().GetTable("");
        singleton->isFMSAttached = true || frc::DriverStation::IsFMSAttached();
    }
        
    return *singleton;
}

void DataLogger::Log( const std::string& s, double val, bool alsoNT ) { SEND_LOG_DATA(s,val,alsoNT) }
void DataLogger::Log( const std::string& s, std::span<const double> a, bool alsoNT ) { SEND_LOG_DATA(s,a,alsoNT) }
void DataLogger::Log( const std::string& s, int val, bool alsoNT ) { int64_t val64{val}; SEND_LOG_DATA(s,val64,alsoNT) }
void DataLogger::Log( const std::string& s, int64_t val, bool alsoNT ) { SEND_LOG_DATA(s,val,alsoNT) }
void DataLogger::Log( const std::string& s, std::span<const int64_t> a, bool alsoNT ) { SEND_LOG_DATA(s,a,alsoNT) }
void DataLogger::Log( const std::string& s, bool val, bool alsoNT ) { SEND_LOG_DATA(s,val,alsoNT) }
void DataLogger::Log( const std::string& s, std::span<const bool> a, bool alsoNT ) { SEND_LOG_DATA(s,a,alsoNT) }
void DataLogger::Log( const std::string& s, const std::string& val, bool alsoNT ) { SEND_LOG_DATA(s,val,alsoNT) }

void DataLogger::Log( const std::string& s, const frc::Pose2d& p, bool alsoNT ) {
    static double a[3];

    a[0] = p.X().value();
    a[1] = p.Y().value();
    a[2] = p.Rotation().Radians().value();

    Log( s, std::span{a}, alsoNT );
}

void DataLogger::Log( const std::string &s, const wpi::array<frc::SwerveModuleState, 4U> &sms, bool alsoNT ) {
    static double a[8];

    if( sms.empty() ) {
        Log( s, std::span<const double>{}, alsoNT );
    } 

    for( int i=0; i<4; ++i ) {
        a[2*i] = sms[i].angle.Radians().value(); 
        a[2*i + 1] = sms[i].speed.value();
    }

    Log( s, std::span{a}, alsoNT );
}

template<> void DataLogger::Log( const std::string &s, const std::vector<double>& vec, bool alsoNT ) {
    GetInstance().Log( s, std::span<const double>( vec ) );
}

/**
 * Pass thru to the frc::DataLogManager::Log() command.
*/
void DataLogger::Log( const std::string &s ) {
    frc::DataLogManager::Log( s );
}





void DataLogger::Send( const std::string& s, double val ) { 
    wpi::log::DoubleLogEntry le{ *(log), s };
    le.Append( val );
}

void DataLogger::Send( const std::string& s, std::span<const double> a ) { 
    wpi::log::DoubleArrayLogEntry le{ *(log), s };
    le.Append( a );
}

void DataLogger::Send( const std::string& s, int64_t val ) { 
    wpi::log::IntegerLogEntry le{ *(log), s };
    le.Append( val );
}

void DataLogger::Send( const std::string& s, std::span<const int64_t> a ) { 
    wpi::log::IntegerArrayLogEntry le{ *(log), s };
    le.Append( a );
}

void DataLogger::Send( const std::string& s, bool val ) {
    wpi::log::BooleanLogEntry le{ *(log), s };
    le.Append( val );
}

void DataLogger::Send( const std::string& s, std::span<const bool> a ) {
    wpi::log::BooleanArrayLogEntry le{ *(log), s };
    le.Append( a );
}

void DataLogger::Send( const std::string& s, const std::string& val ) { 
    wpi::log::StringLogEntry le{ *(log), s };
    le.Append( val );
}





void DataLogger::SendNT( const std::string& s, double val ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetDoubleTopic( s ).GenericPublish( "double" );
    }
    nt_map[s].SetDouble( val );
}

void DataLogger::SendNT( const std::string& s, std::span<const double> a ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetDoubleArrayTopic( s ).GenericPublish( "double[]" );
    }
    nt_map[s].SetDoubleArray( a );
}

void DataLogger::SendNT( const std::string &s, int64_t val ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetIntegerTopic( s ).GenericPublish( "integer" );
    }
    nt_map[s].SetInteger( val );
}

void DataLogger::SendNT( const std::string& s, std::span<const int64_t> a ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetIntegerArrayTopic( s ).GenericPublish( "integer[]" );
    }
    nt_map[s].SetIntegerArray( a );
}

void DataLogger::SendNT( const std::string &s, bool val ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetBooleanTopic( s ).GenericPublish( "boolean" );
    }
    nt_map[s].SetBoolean( val );
}

void DataLogger::SendNT( const std::string& s, std::span<const bool> a ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetBooleanArrayTopic( s ).GenericPublish( "boolean[]" );
    }
    nt_map[s].SetBooleanArray( a );
}

void DataLogger::SendNT( const std::string &s, const std::string &val ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetStringTopic( s ).GenericPublish( "string" );
    }
    nt_map[s].SetString( val );
}



// void DataLogger::SendNT( std::string s, frc::Trajectory &t ) {
//     std::vector<double> a;

//     for( auto &&s : t.States() ) {
//         a.push_back(s.pose.X().value());
//         a.push_back(s.pose.Y().value());
//         a.push_back(s.pose.Rotation().Degrees().value());
//     }
//     SendNT( s, a );
// }

void DataLogger::LogMetadata( void ) {
        // Open the buildinfo.txt file and write the Metadata to the log file
    std::ifstream binfo;
    char line[256];

    binfo.open( frc::filesystem::GetDeployDirectory() + "/buildinfo.txt", std::ios::in );
    if( binfo.is_open() ) {
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "BUILD_DATE", line );
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "GIT_REPO", line );
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "GIT_BRANCH", line );
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "GIT_VERSION", line );
        binfo.close();
    }

}

void DataLogger::SendMetadata( std::string_view s, std::string_view val ) {
        // AdvantageScope Chops off leading Character of the name so we add an underscore.
        // Not sure why
    std::string id = "RealMetadata/_";
    id += s;
    wpi::log::StringLogEntry le{ *(log), id };
    le.Append( val );
}
