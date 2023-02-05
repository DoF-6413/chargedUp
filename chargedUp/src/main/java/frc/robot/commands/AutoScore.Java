// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoScore extends SequentialCommandGroup {
  /** Creates a new AutoScore.//Eventually this is where we will put the autoscore command after five seconds */
  public AutoScore() {
    addCommands(
        new WaitCommand(2),
        // Todo fix this error v
        //new RunCommand( SmartDashboard.putString("hit", "wait command running") )
        
    );
  }
}