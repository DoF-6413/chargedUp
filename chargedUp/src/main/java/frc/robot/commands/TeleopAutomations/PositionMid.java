// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutomations;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.TelescoperConstants;
import frc.robot.commands.ArmControls.RotationPID;
import frc.robot.commands.ArmControls.TelescoperReset;
import frc.robot.commands.ArmControls.TelescoperWrapper;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PositionMid extends SequentialCommandGroup {
  /** Creates a new PositionMid. */
  public PositionMid(TelescoperSubsystem telescoper, ArmSubsystem arm, EndEffectorSubsystem NEfector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TelescoperReset(telescoper),
      new RotationPID(arm, -ArmConstants.kHPMPHB),
      new TelescoperWrapper(telescoper, arm, NEfector, TelescoperConstants.kMCGB)
    );
  }
}
