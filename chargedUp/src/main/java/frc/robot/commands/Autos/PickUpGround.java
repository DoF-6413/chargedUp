// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;
import frc.robot.Constants.TelescoperConstants;
import frc.robot.commands.ArmControls.EndEffectorRunner;
import frc.robot.commands.ArmControls.RotationPID;
import frc.robot.commands.ArmControls.TelescoperPID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpGround extends SequentialCommandGroup {
  private TelescoperSubsystem telescoper;
  private EndEffectorSubsystem endEffector;
  private ArmSubsystem arm;
  /** Creates a new PickUpGround. */
  public PickUpGround(ArmSubsystem m_armSubsystem, TelescoperSubsystem m_telescoperSubsystem, EndEffectorSubsystem m_endEffectorSubsystem ) {
    telescoper = m_telescoperSubsystem;
    endEffector = m_endEffectorSubsystem;
    arm = m_armSubsystem;
    addRequirements(telescoper, endEffector, arm);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
       //take arm out 
    new RotationPID(arm, 45),
    //extend telescoper 
    new TelescoperPID(telescoper, 130),
    //intake endeffector
    new EndEffectorRunner(endEffector, .5, 3),
    //pull back telescoper
    new TelescoperPID(telescoper, 0),
    //bring in arm 
    new RotationPID(arm, 0)
    );
  }
}
