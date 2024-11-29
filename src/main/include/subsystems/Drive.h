#pragma once

#include <wpi/array.h>
#include <frc/geometry/Pose3d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

#include <units/angular_velocity.h>

class Drive : public frc2::SubsystemBase {
public:
    Drive();
    void ArcadeDrive( double xAxis, double yAxis, double omegaAxis );

    const frc::Pose3d& GetPose() { return inputs.pose3d; }

    frc2::CommandPtr ArcadeDriveCmd(
        std::function<double()> xSupplier, 
        std::function<double()> ySupplier, 
        std::function<double()> omegaSupplier
    );

    struct Inputs {
        frc::Pose3d pose3d;
        wpi::array<frc::SwerveModuleState, 4U> swerveStates{wpi::empty_array};
        frc::Field2d field;

        void Log( std::string key );
    };

private:
    Inputs inputs;
    frc::SwerveDriveKinematics<4> kinematics;
};

