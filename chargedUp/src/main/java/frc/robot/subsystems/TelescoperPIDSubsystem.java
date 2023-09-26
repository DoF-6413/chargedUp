// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class TelescoperPIDSubsystem extends ProfiledPIDSubsystem {
    private final CANSparkMax telescoperMotorSparkMax;
    private final RelativeEncoder telescoperEncoder; 

    public TelescoperPIDSubsystem() {

    }


}