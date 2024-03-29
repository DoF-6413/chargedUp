// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmControls.EndEffectorRunner;
import frc.robot.commands.ArmControls.RotationPID;
import frc.robot.commands.ArmControls.TelescoperPID;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackingOutArm extends SequentialCommandGroup {
  /** Creates a new BackingOutArm. */
  public 
  BackingOutArm(ArmSubsystem arm, TelescoperSubsystem telescoper, EndEffectorSubsystem NEfctr) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new EndEffectorRunner(NEfctr, -0.8, 0.5),
        new TelescoperPID(telescoper, 1)
      ),
      new RotationPID(arm, 0)
    );
  }
}
