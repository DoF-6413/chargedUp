// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutomations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Autos.BackingOutArm;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterBackIn extends SequentialCommandGroup {
  /** Creates a new ShooterBackIn. */
  public ShooterBackIn(ArmPIDSubsystem arm, TelescoperSubsystem telescoper, EndEffectorSubsystem NEFector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.runOnce(
            () -> {
              arm.updateAcceleration(3);
            },
            arm),
            new WaitUntilCommand(()-> arm.atGoal()),
            new BackingOutArm(arm, telescoper, NEFector),
            Commands.runOnce(
             () -> {
               arm.updateAcceleration(7);
             } )

    );
  }
}