// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.TelescoperConstants;

public class TelescoperPIDSubsystem extends ProfiledPIDSubsystem {
    private final CANSparkMax telescoperMotorSparkMax;
    private final RelativeEncoder telescoperEncoder;

    public TelescoperPIDSubsystem() {
        super(
            new ProfiledPIDController (
                TelescoperConstants.kTelescoperP,
                TelescoperConstants.kTelescoperI,
                TelescoperConstants.kTelescoperD, null);
        )

        telescoperMotorSparkMax = new CANSparkMax(telescoperMotorSparkMax, MotorType.kBrushless);
        telescoperMotorSparkMax.setSmartCurrentLimit(60);

    }

    controller.setTolerance(TelescoperConstants.kRotationTolerance);
    setGoal(TelescoperConstants.kTelescoperOffsetRad);
}

