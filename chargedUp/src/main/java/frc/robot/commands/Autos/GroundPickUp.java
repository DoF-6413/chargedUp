// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TelescoperConstants;
import frc.robot.commands.ArmControls.EndEffectorRunner;
import frc.robot.commands.ArmControls.RotationPID;
import frc.robot.commands.ArmControls.TelescoperPID;
import frc.robot.commands.ArmControls.TelescoperReset;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundPickUp extends SequentialCommandGroup {
  /** Creates a new GroundPickUp. */
  public GroundPickUp(TelescoperSubsystem telscoper, ArmSubsystem arm, EndEffectorSubsystem NEfector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TelescoperReset(telscoper),
      new RotationPID(arm, ArmConstants.kfloorCube),
      new TelescoperPID(telscoper, TelescoperConstants.kMCGB),
      new ParallelCommandGroup(
        new EndEffectorRunner(NEfector, 0.5, 1),
        new TelescoperPID(telscoper, TelescoperConstants.kGroundCone)),
      new RotationPID(arm, 40),
      new TelescoperPID(telscoper, 0),
      new RotationPID(arm, 0)

    );
  }
}
