#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"

ClimberSubsystem::ClimberSubsystem() {
    // Additional initialization if needed
}

void ClimberSubsystem::extendPiston() {    
     m_doubleSolenoid.Set(frc::DoubleSolenoid::kOff);
     m_doubleSolenoid.Set(frc::DoubleSolenoid::kForward);
     m_doubleSolenoid2.Set(frc::DoubleSolenoid::kOff);
     m_doubleSolenoid2.Set(frc::DoubleSolenoid::kForward);
}

void ClimberSubsystem::retractPiston() {    
     m_doubleSolenoid.Set(frc::DoubleSolenoid::kOff);
     m_doubleSolenoid.Set(frc::DoubleSolenoid::kReverse);
     m_doubleSolenoid2.Set(frc::DoubleSolenoid::kOff);
     m_doubleSolenoid2.Set(frc::DoubleSolenoid::kReverse);
}