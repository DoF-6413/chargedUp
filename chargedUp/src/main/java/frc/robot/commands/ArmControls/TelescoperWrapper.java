// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmControls;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TelescoperConstants;
import frc.robot.commands.TeleopAutomations.BackIn;
import frc.robot.commands.TeleopAutomations.PlaceHighCone;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TelescoperWrapper extends PIDCommand {
  /** Creates a new TelescoperWrapper. */
  private TelescoperSubsystem m_telescoperSubsystem;
  private ArmPIDSubsystem m_armSubsystem;
  private EndEffectorSubsystem m_endEffectorSubsystem;

  public TelescoperWrapper(TelescoperSubsystem telescope, ArmPIDSubsystem arm, EndEffectorSubsystem NEfector, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(TelescoperConstants.kTelescoperP, TelescoperConstants.kTelescoperI, TelescoperConstants.kTelescoperD),
        // This should return the measurement
        () -> telescope.getTelescoperPosition(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> { telescope.spinTelescopingMotor(output);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    m_telescoperSubsystem = telescope;
    m_armSubsystem = arm;
    m_endEffectorSubsystem = NEfector;
    SmartDashboard.putNumber("Telescoper Setpoint", setpoint);
    addRequirements(m_telescoperSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(TelescoperConstants.kTelescoperTolerance);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return CommandScheduler.getInstance().isScheduled(new PlaceHighCone(m_armSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem)) || 
    CommandScheduler.getInstance().isScheduled(new BackIn(m_telescoperSubsystem, m_armSubsystem));
    // return false;
  }
}
