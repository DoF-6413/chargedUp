// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutomations;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.commands.ArmControls.RotationPID;
import frc.robot.commands.Autos.BackingOutArm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceMid extends SequentialCommandGroup {
  /** Creates a new PlaceMid. */
  public PlaceMid(ArmSubsystem arm, TelescoperSubsystem telescoper, EndEffectorSubsystem NEFector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotationPID(arm, -ArmConstants.kMidBottom),
      new BackingOutArm(arm, telescoper, NEFector)
    );
  }
}
