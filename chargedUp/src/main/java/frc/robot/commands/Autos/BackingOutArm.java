// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmControls.EndEffectorRunner;
// import frc.robot.commands.ArmControls.TelescoperPID;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperPIDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackingOutArm extends SequentialCommandGroup {
  /** Creates a new BackingOutArm. */
  public 
  BackingOutArm(ArmPIDSubsystem arm, TelescoperPIDSubsystem telescoper, EndEffectorSubsystem NEfctr) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new EndEffectorRunner(NEfctr, -0.3, 0.25),
        Commands.runOnce(
          ()-> {
            telescoper.setGoal(1);
            telescoper.enable();
          },
        telescoper)
      ),
      new WaitUntilCommand(()-> telescoper.atGoal()),
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
