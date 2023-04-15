// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutomations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.TelescoperConstants;

import frc.robot.commands.ArmControls.TelescoperReset;
import frc.robot.commands.ArmControls.TelescoperWrapper;
import frc.robot.commands.ArmControls.WristPID;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PositionMid extends SequentialCommandGroup {
  /** Creates a new PositionMid. */
  public PositionMid(TelescoperSubsystem telescoper, ArmPIDSubsystem arm, EndEffectorSubsystem NEfector, WristSubsystem wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TelescoperReset(telescoper),
      Commands.runOnce(
        () -> {
          arm.setGoal(Units.degreesToRadians(-ArmConstants.kHPMP)+ArmConstants.kArmOffsetRads);
          arm.enable();
        },
        arm),
        new WaitUntilCommand(()-> arm.atGoal()),
      new ConditionalCommand(
        // new TelescoperWrapper(telescoper, arm, NEfector, TelescoperConstants.kMCGB)
      //   new ParallelCommandGroup(
          new WristPID(wrist, 0),
          new TelescoperWrapper(telescoper, arm, NEfector, TelescoperConstants.kMCGB),
        ()-> (wrist.getPosition() > -30 && wrist.getPosition() < 30) || (wrist.getPosition() > -210 && wrist.getPosition() < -160)
      )
    );
  }
}
