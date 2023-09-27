// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.TelescoperConstants;


public class TelescoperPIDSubsystem extends ProfiledPIDSubsystem {
    private final TalonFX telescoperMotor;
    private final RelativeEncoder telescoperEncoder;

    public TelescoperPIDSubsystem() {
        new ProfiledPIDController (
            TelescoperConstants.kTelescoperP,
            TelescoperConstants.kTelescoperI,
            TelescoperConstants.kTelescoperD, 
            null);

        telescoperMotor = new TalonFX(TelescoperConstants.kTelescoperCANID);
        StatorCurrentLimitConfiguration currentLimitConfig = new StatorCurrentLimitConfiguration();
    }

    m_controller.setTolerance(TelescoperConstants.kRotationTolerance);


	@Override
	protected void useOutput(double output, State setpoint) {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected double getMeasurement() {
		// TODO Auto-generated method stub
		return 0;
	}}

