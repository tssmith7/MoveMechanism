#pragma once

#include <string>
#include <vector>

#include "motor/SimMotorIO.h"
#include "subsystems/ExampleIO.h"

#include <frc/Timer.h>

class ExampleIOSim : public ExampleIO {
public:
    virtual void UpdateInputs( ExampleSubsysInputs &inputs );

    virtual void SetArmGoal( units::degree_t goal ) { m_armMotor.SetMotionControl( goal ); }
    virtual void SetWristGoal( units::degree_t goal ) { m_wristMotor.SetMotionControl( goal ); }
    virtual void SetElevGoal( units::inch_t goal ) { m_elevMotor.SetMotionControl( goal ); }

private:
    MotionConfig<units::degrees> armMC = { {0.1,0,0}, {1_tps, 4_tr_per_s_sq, 0_tr_per_s_cu} };
    MotionConfig<units::degrees> wristMC = { {0.1,0,0}, {1_tps, 4_tr_per_s_sq, 0_tr_per_s_cu} };
    MotionConfig<units::inch> elevMC = { {0.05,0,0}, {2_mps, 4_mps_sq, 0_mps_cu} };
    
    SimMotorIO<units::degrees> m_armMotor{ 1, 2.0, armMC };
    SimMotorIO<units::degrees> m_wristMotor{ 1, 1.0, wristMC  };
    SimMotorIO<units::inch> m_elevMotor{ 1, 2.0, elevMC };
};

void ExampleIOSim::UpdateInputs( ExampleSubsysInputs &inputs ) {
    static units::second_t nextTime = 10_s;
    m_armMotor.Update();
    m_wristMotor.Update();
    m_elevMotor.Update();

    MotorIO<units::degrees>::Inputs inp_deg;
    inp_deg = m_armMotor.GetInputs();
    inputs.armPositionHistory[1] = inputs.armPosition;
    inputs.armPositionHistory[0] = inp_deg.position;


    if( frc::Timer::GetFPGATimestamp() > 10_s ) {
        if( frc::Timer::GetFPGATimestamp() > nextTime ) {
            nextTime += 1_s;
            for(int i=9; i>0; --i ) {
                inputs.oneSecInc[i] = inputs.oneSecInc[i-1];
            }
            inputs.oneSecInc[0] = true;
        }
        inputs.armPositionHistoryDbl = {};
        inputs.visionPose.reset();
        inputs.armSetpoint.reset();
        inputs.has10selasped = true;
    } else {
        inputs.armPositionHistoryDbl[1] = inputs.armPosition.value();
        inputs.armPositionHistoryDbl[0] = inp_deg.position.value();
        inputs.visionPose = { 1_m, 2_m + (inputs.armPositionHistoryDbl[1]-inputs.armPositionHistoryDbl[0])*1_m, 0_deg };
        inputs.armSetpoint = inputs.armPositionHistory[0];
    }
 
    inputs.armPosition = inp_deg.position;
    inputs.armVelocity = inp_deg.velocity;
    inputs.armAppliedVolts = inp_deg.inputVoltage;
    inputs.armCurrent = inp_deg.currentDraw;

    inp_deg = m_wristMotor.GetInputs();
    inputs.wristPosition = inp_deg.position;
    inputs.wristVelocity = inp_deg.velocity;
    inputs.wristAppliedVolts = inp_deg.inputVoltage;
    inputs.wristCurrent = inp_deg.currentDraw;

    MotorIO<units::inches>::Inputs inp_inch;
    inp_inch = m_elevMotor.GetInputs();
    inputs.elevPosition = inp_inch.position;
    inputs.elevVelocity = inp_inch.velocity;
    inputs.elevAppliedVolts = inp_inch.inputVoltage;
    inputs.elevCurrent = inp_inch.currentDraw;
}
