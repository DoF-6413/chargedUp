// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutomations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmControls.EndEffectorRunner;
import frc.robot.commands.ArmControls.TelescoperPID;
import frc.robot.commands.ArmControls.TelescoperReset;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundPickUp extends SequentialCommandGroup {
  /** Creates a new GroundPickUp. */
  public GroundPickUp(TelescoperSubsystem telscoper, ArmPIDSubsystem arm, EndEffectorSubsystem NEfector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TelescoperReset(telscoper),
      Commands.runOnce(
        () -> {
          arm.setGoal(Units.degreesToRadians(ArmConstants.kfloorCube)+ArmConstants.kArmOffsetRads);
          arm.enable();
        },
        arm),
        new WaitUntilCommand(()-> arm.atGoal()),
        new TelescoperPID(telscoper, 28),
      // new ParallelCommandGroup(
        new EndEffectorRunner(NEfector, 0.5, 5),
        new BackIn(telscoper, arm)
        // new TelescoperWrapper(telscoper, arm, NEfector, TelescoperConstants.kGroundCone))
        // new EndEffectorRunner(NEfector, 0.5, 10)
    );
  }
}
