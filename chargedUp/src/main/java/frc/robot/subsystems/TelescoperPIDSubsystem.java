// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.TelescoperConstants;


public class TelescoperPIDSubsystem extends ProfiledPIDSubsystem {
    private final WPI_TalonFX telescoperMotor;
    
    public TelescoperPIDSubsystem() {
        super(
            new ProfiledPIDController (
                TelescoperConstants.kTelescoperP,
                TelescoperConstants.kTelescoperI,
                TelescoperConstants.kTelescoperD, 
                new TrapezoidProfile.Constraints(
                    TelescoperConstants.kTelescoperMaxVelocity,
                    TelescoperConstants.kTelescoperMaxAcceleration)));

        telescoperMotor = new WPI_TalonFX(TelescoperConstants.kTelescoperCANID);
        StatorCurrentLimitConfiguration telescopCurrentLimitConfiguration = new StatorCurrentLimitConfiguration(
            TelescoperConstants.kIsTelescoperCurrentLimitEnabled,
            TelescoperConstants.kTelescoperContinuousCurrent,
            TelescoperConstants.kTelescoperPeakCurrent,
            TelescoperConstants.kTelescoperMaxTimeAtPeak);
        telescoperMotor.configStatorCurrentLimit(telescopCurrentLimitConfiguration);
        //position conversion factor
        telescoperMotor.configSelectedFeedbackCoefficient(TelescoperConstants.kTelescopePositionConversionFactor);

        m_controller.setTolerance(TelescoperConstants.kTelescoperTolerance);
        setGoal(TelescoperConstants.kTelescoperOffset);
    }
            

    
    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return telescoperMotor.getSelectedSensorPosition() + TelescoperConstants.kTelescoperOffset;
    }
    
    @Override
    protected void useOutput(double output, State setpoint) {
        // TODO Auto-generated method stub
        telescoperMotor.setVoltage(output);
    }

    //copy over methods from telescopersubsystem
    public void updateGoal(double increment) {
        //do this
    }
}

