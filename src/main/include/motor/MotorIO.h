#pragma once

#include <string>
#include <vector>

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>

    // Create the jerk unit meters_per_second_cubed
namespace units {
UNIT_ADD(jerk, meters_per_second_cubed, meters_per_second_cubed,
         mps_cu, compound_unit<length::meter, inverse<cubed<time::seconds>>>)
}

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>
#include <units/acceleration.h>

/**
 * Controller Parameters kP, kI, kD, kS, kG, kV, kA
*/
struct TuningParams {
    double kP;
    double kI;
    double kD;
    double kS;
    double kG;
    double kV;
    double kA;
};

template<class Distance> struct MotionParams {
    using Distance_t = units::unit_t<Distance>;
    using Velocity =
        units::compound_unit<Distance, units::inverse<units::seconds>>;
    using Velocity_t = units::unit_t<Velocity>;
    using Acceleration =
        units::compound_unit<Velocity, units::inverse<units::seconds>>;
    using Acceleration_t = units::unit_t<Acceleration>;
    using Jerk =
        units::compound_unit<Acceleration, units::inverse<units::seconds>>;
    using Jerk_t = units::unit_t<Jerk>;

    Velocity_t MaxVelocity;
    Acceleration_t MaxAcceleration;
    Jerk_t MaxJerk;
};

template<class Distance> struct MotionConfig {
    TuningParams tuner;
    MotionParams<Distance> mp;
};

struct MotorConfigs {
    std::string name = "Unknown";
    int canId = -1;
    std::string canBus = "";
    bool counterClockwisePositive = true;
    std::vector<std::pair<int,bool>> followers;
};


template<class Distance> class MotorIO {
public:
    using Distance_t = units::unit_t<Distance>;
    using Velocity =
        units::compound_unit<Distance, units::inverse<units::seconds>>;
    using Velocity_t = units::unit_t<Velocity>;

    struct Inputs {
        Distance_t position;
        Velocity_t velocity;
        units::volt_t inputVoltage;
        units::ampere_t currentDraw;
    };

    virtual void SetOpenLoop( double percentOutput ) = 0;
    virtual void SetMotionControl( Distance_t goal ) = 0;
    virtual void SetPosition( Distance_t position ) = 0;
    virtual Inputs &GetInputs() = 0;
    virtual void SetCoastMode( bool coast ) = 0;
    virtual void Nudge( Distance_t delta ) = 0;
};
