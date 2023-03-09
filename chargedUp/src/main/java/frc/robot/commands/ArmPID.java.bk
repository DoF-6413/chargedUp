// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmPID extends PIDCommand {
  /** Creates a new ArmPID. */
  private ArmSubsystem m_armSubsystem;
  public ArmPID(ArmSubsystem arm, double targetPosition) {
    super(
        // The controller that the command will use
        new PIDController(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD),
        // This should return the measurement
        () -> arm.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> targetPosition,
        // This uses the output
        output -> { 
          // Use the output here
          arm.spinMotor(output); //Divides by 180 to make output in between -1 and 1
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_armSubsystem = arm;
    addRequirements(m_armSubsystem);

    getController().setTolerance(ArmConstants.kArmTolerance);
  }

  @Override
  public void initialize() {
    System.out.println("command running");
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (getController().atSetpoint() == true);
  }
}
