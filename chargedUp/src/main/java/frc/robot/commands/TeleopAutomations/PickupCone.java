// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutomations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class PickupCone extends SequentialCommandGroup {
  /** Creates a new PickupCone. */
  public PickupCone(ArmPIDSubsystem arm, TelescoperSubsystem telescoper, EndEffectorSubsystem NEffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
    new TelescoperReset(telescoper),
      Commands.runOnce(
            () -> {
              arm.setGoal(Units.degreesToRadians(ArmConstants.kHPMPHB-ArmConstants.kArmOffsetRads));
              arm.enable();
            },
            arm),
      new EndEffectorRunner(NEffector, 0.5, 5)
    );
  }
}
