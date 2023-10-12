// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CamAutos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TelescoperConstants;
import frc.robot.commands.ArmControls.EndEffectorRunner;
import frc.robot.commands.ArmControls.TelescoperPID;
import frc.robot.commands.ArmControls.TelescoperReset;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreLeaveArm extends SequentialCommandGroup {
  /** Creates a new ScoreLeaveArm. */
  public ScoreLeaveArm(ArmPIDSubsystem arm, TelescoperSubsystem telescoper, EndEffectorSubsystem NEfctr, DrivetrainSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TelescoperReset(telescoper),
      Commands.runOnce(
        () -> {
          arm.setGoal(Units.degreesToRadians(-ArmConstants.kHighScoreInitial )+ArmConstants.kArmOffsetRads);
          arm.enable();
        },
        arm),
        new WaitUntilCommand(()-> arm.atGoal()),
      new TelescoperPID(telescoper, TelescoperConstants.kMaxExtention),
      Commands.runOnce(
        () -> {
          arm.setGoal(Units.degreesToRadians(-ArmConstants.kHighScoreFinal)+ArmConstants.kArmOffsetRads);
          arm.enable();
        },
        arm),
        new WaitUntilCommand(()-> arm.atGoal()),
      new EndEffectorRunner(NEfctr, -0.8, 0.5)
            
    );
  }
}
