
#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>

#include "subsystems/Drive.h"

#include "DataLogger.h"

const units::inch_t kDriveBaseLength = 27_in;
const units::inch_t kDriveBaseWidth = 27_in;

Drive::Drive() :
    kinematics{ frc::Translation2d{+( kDriveBaseLength / 2 ), +( kDriveBaseWidth / 2 )},
                frc::Translation2d{+( kDriveBaseLength / 2 ), -( kDriveBaseWidth / 2 )},
                frc::Translation2d{-( kDriveBaseLength / 2 ), +( kDriveBaseWidth / 2 )},
                frc::Translation2d{-( kDriveBaseLength / 2 ), -( kDriveBaseWidth / 2 )} }

{ 
    frc::SmartDashboard::PutData( &inputs.field ); 
}

void Drive::ArcadeDrive( double xAxis, double yAxis, double omegaAxis )
{
    units::meter_t deltax = frc::ApplyDeadband<double>(xAxis, 0.1) * 0.1_m;
    units::meter_t deltay = frc::ApplyDeadband<double>(yAxis, 0.1) * 0.1_m;
    units::radian_t deltar = frc::ApplyDeadband<double>(omegaAxis, 0.1) * 0.1_rad;
    units::radian_t planeAngle = inputs.pose3d.Rotation().Z();

        // Correct for robot orientation and axis reversal.  Setup so AdvantageScope controls are screen oriented.
    
    frc::Transform3d controllerDelta = frc::Transform3d( 
        { units::math::cos(planeAngle) * deltax - units::math::sin(planeAngle) * deltay, 
          -units::math::sin(planeAngle) * deltax + units::math::cos(planeAngle) * -deltay, 0_m}, 
          {0_deg, 0_deg, deltar} );

    frc::ChassisSpeeds chassisSpeed = { controllerDelta.X() / 20_ms, controllerDelta.Y() / 20_ms, controllerDelta.Rotation().Z() / 20_ms };

    inputs.swerveStates = kinematics.ToSwerveModuleStates( chassisSpeed );

    inputs.pose3d = inputs.pose3d + controllerDelta;

    inputs.Log( "Drive" );
}

void Drive::Inputs::Log( std::string key )
{
    AUTOLOG( key, pose3d );
    AUTOLOG( key, swerveStates );
}

frc2::CommandPtr Drive::ArcadeDriveCmd(
    std::function<double()> xSupplier, 
    std::function<double()> ySupplier, 
    std::function<double()> omegaSupplier )
{
    return frc2::cmd::Run( [this, xSupplier, ySupplier, omegaSupplier] {
        ArcadeDrive( xSupplier(), ySupplier(), omegaSupplier() );
    }, {this} );

}