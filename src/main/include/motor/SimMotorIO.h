#pragma once

#include <units/moment_of_inertia.h>

#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include "motor/MotorIO.h"

template<class Distance> class SimMotorIO : public MotorIO<Distance> {
public:
    using Distance_t = units::unit_t<Distance>;

    SimMotorIO( double mechRatio, double sluggishness, MotionConfig<Distance> mc );

    virtual void SetOpenLoop( double percentOutput ) { inputs.inputVoltage = percentOutput * 12_V; m_motor.SetInputVoltage( inputs.inputVoltage );  }
    virtual void SetMotionControl( Distance_t goal ) { m_Goal.position = goal; }
    virtual void SetPosition( Distance_t position );
    virtual MotorIO<Distance>::Inputs &GetInputs();
    virtual void SetCoastMode( bool coast ){}
    virtual void Nudge( Distance_t delta ) { m_Goal.position += delta; }

    void Update();

private:
    double m_mechRatio;
    frc::sim::DCMotorSim m_motor;
    MotorIO<Distance>::Inputs inputs;

    inline static const units::kilogram_square_meter_t m_baseMOI = 0.0005_kg_sq_m;

    frc::PIDController m_softPID;
    frc::SimpleMotorFeedforward<Distance> *m_motorFF;
    frc::TrapezoidProfile<Distance> m_Profile;
    frc::TrapezoidProfile<Distance>::State m_Goal;
    frc::TrapezoidProfile<Distance>::State m_Setpoint;
};


template<class Distance> 
SimMotorIO<Distance>::SimMotorIO( double mechRatio, double sluggishness,  MotionConfig<Distance> mc )
    : m_motor{ frc::DCMotor::KrakenX60(1), 1.0, SimMotorIO<Distance>::m_baseMOI * sluggishness }, 
      m_Profile{ {mc.mp.MaxVelocity, mc.mp.MaxAcceleration} }, 
      m_softPID{ mc.tuner.kP, mc.tuner.kI, mc.tuner.kD },
      m_mechRatio{mechRatio} 
{
    m_softPID.SetPID( mc.tuner.kP, mc.tuner.kI, mc.tuner.kD );

    m_motorFF = new frc::SimpleMotorFeedforward<Distance>{ 
        units::volt_t{mc.tuner.kS},
        units::unit_t<frc::SimpleMotorFeedforward<Distance>::kv_unit>{mc.tuner.kV}, 
        units::unit_t<frc::SimpleMotorFeedforward<Distance>::ka_unit>{mc.tuner.kA}
    };
    m_Goal = {MotionParams<Distance>::Distance_t(0), MotionParams<Distance>::Velocity_t(0) };
}

template<class Distance> 
void SimMotorIO<Distance>::SetPosition( Distance_t position ) { 
    inputs.position = position;
    m_motor.SetState( units::radian_t(position.value() / m_mechRatio), 0_rad_per_s ); 
}

template<class Distance> 
typename MotorIO<Distance>::Inputs &SimMotorIO<Distance>::GetInputs() { 
    return inputs; 
}

template<class Distance> 
void SimMotorIO<Distance>::Update() {

    typename frc::TrapezoidProfile<Distance>::State currState;

    currState = { inputs.position, inputs.velocity };

    m_Setpoint = m_Profile.Calculate( 20_ms, m_Setpoint, m_Goal);
    
    double pidOut = m_softPID.Calculate( currState.position.value(), m_Setpoint.position.value() );
    units::volt_t ffOut = m_motorFF->Calculate( m_Setpoint.velocity );

    inputs.inputVoltage = pidOut * 12_V + ffOut;
    m_motor.SetInputVoltage( inputs.inputVoltage );

    m_motor.Update( 20_ms );

    inputs.position = Distance_t(m_motor.GetAngularPosition().value() * m_mechRatio); 
    inputs.velocity = MotorIO<Distance>::Velocity_t(m_motor.GetAngularVelocity().value() * m_mechRatio);
    inputs.currentDraw = m_motor.GetCurrentDraw();

    // fmt::print( "Motor, goal={:.5}, setpt={:.5}, curr={:.5},  V={:.5}\n", 
    //     m_Goal.position, m_Setpoint.position.value(), currState.position.value(), inputVoltage );
}