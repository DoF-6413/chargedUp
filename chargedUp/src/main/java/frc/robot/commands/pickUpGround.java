// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;
import frc.robot.Constants.TelescoperConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmControls.*;

public class pickUpGround extends CommandBase {
  private TelescoperSubsystem telescoper;
  private EndEffectorSubsystem endEffector;
  private ArmSubsystem arm;
  private static boolean stopCommand = false;
 
  /** Creates a new pickUpGround. */
  public pickUpGround(ArmSubsystem m_armSubsystem, TelescoperSubsystem m_telescoperSubsystem, EndEffectorSubsystem m_endEffectorSubsystem ) {
    // Use addRequirements() here to declare subsystem dependencies.
    telescoper = m_telescoperSubsystem;
    endEffector = m_endEffectorSubsystem;
    arm = m_armSubsystem;
    addRequirements(telescoper, endEffector, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    new SequentialCommandGroup(
    //take arm out 
    new RotationPID(arm, 40),
    //extend telescoper 
    new TelescoperPID(telescoper, 10),
    //intake endeffector
    new EndEffectorRunner(endEffector, .5, 3),
    //pull back telescoper
    new TelescoperPID(telescoper, 0),
    //bring in arm 
    new RotationPID(arm, 0)
     );
     stopCommand = true;
  }
//  public void stop(){
//   stopCommand = true;
//  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopCommand;
  }
}
