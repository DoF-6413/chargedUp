// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CamAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToCone;
import frc.robot.commands.ArmControls.EndEffectorRunner;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CamPickUpCone extends SequentialCommandGroup {
  /** Creates a new CamPickUpCone. */
  public CamPickUpCone(DrivetrainSubsystem m_DrivetrainSubsystem, VisionSubsystem m_VisionSubsystem, TelescoperSubsystem m_TelescoperSubsystem, ArmPIDSubsystem m_ArmPIDSubsystem, EndEffectorSubsystem m_EffectorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreLeaveArm(m_ArmPIDSubsystem, m_TelescoperSubsystem, m_EffectorSubsystem, m_DrivetrainSubsystem),
      new ParallelCommandGroup(
      new DriveToCone(m_DrivetrainSubsystem, m_VisionSubsystem),
      new ReturnArm(m_ArmPIDSubsystem, m_TelescoperSubsystem, m_EffectorSubsystem, m_DrivetrainSubsystem)),
      new ParallelDeadlineGroup(
        new FirstHalfGroundPickUp(m_TelescoperSubsystem, m_ArmPIDSubsystem, m_EffectorSubsystem),
        new RunCommand(()-> m_DrivetrainSubsystem.setRaw(0.27, 0))),
        new SecondHalfGroundPickup(m_TelescoperSubsystem, m_ArmPIDSubsystem, m_DrivetrainSubsystem, m_EffectorSubsystem),
      new EndEffectorRunner(m_EffectorSubsystem, 0.5, 1)
      //TODO: Put extra stuff here (Return to Grid and Score high)
    );
  }
}
