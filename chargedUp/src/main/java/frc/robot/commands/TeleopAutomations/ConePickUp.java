// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutomations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmControls.TelescoperPID;
import frc.robot.commands.ArmControls.WristPID;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConePickUp extends SequentialCommandGroup {
  /** Creates a new ConePickUp. */
  public ConePickUp(WristSubsystem wrist, TelescoperSubsystem telerescoper ,ArmPIDSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new ConditionalCommand(
          new WristPID(wrist, -180),
          new WristPID(wrist, 0), 
          () -> (wrist.getPosition() > -30 && wrist.getPosition() < 30)),
       
        new TelescoperPID(telerescoper, 0)
      ),
      Commands.runOnce(
        () -> {
          arm.setGoal(Units.degreesToRadians(0)+ArmConstants.kArmOffsetRads);
          arm.enable();
        },
        arm),
        new WaitUntilCommand(()-> arm.atGoal())
    );
  }
}
