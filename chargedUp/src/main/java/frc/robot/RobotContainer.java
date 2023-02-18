// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.commands.AutoScore2;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drivetotag;
import frc.robot.commands.locateCube;
// import frc.robot.commands.targetFinding;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.commands.ArmPID;
// import frc.robot.commands.targetFinding;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.commands.ArmPID;
import frc.robot.commands.ArmControls.RotationPID;
import frc.robot.commands.DrivetrainControls.MovePID;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import javax.swing.plaf.basic.BasicOptionPaneUI.ButtonActionListener;
import edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();


  // list of autos
  private final Command m_autoScore = new AutoScore2();


  // private final Command m_driveToTag = new drivetotag(m_drivetrainSubsystem, m_visionSubsystem); 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final static XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);
      public Trigger driverLeftTrigger = new Trigger(() -> ArmSubsystem.getLeftTriggerActive());
      public Trigger driverRightTrigger = new Trigger(() -> ArmSubsystem.getRightTriggerActive());

      public final static CommandXboxController m_auxController =
      new CommandXboxController(3);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SendableChooser<Command> m_chooser = new SendableChooser<>();
  public RobotContainer() {
    // Configure the trigger bindings

    
    m_chooser.setDefaultOption("Auto Score", m_autoScore);
    m_chooser.addOption("Auto Score", m_autoScore);
      SmartDashboard.putData(m_chooser);
      
    m_drivetrainSubsystem.setDefaultCommand(new RunCommand(() ->
     m_drivetrainSubsystem.setRaw(m_driverController.getLeftY(), m_driverController.getRightX()), m_drivetrainSubsystem));
    configureBindings();


    // m_chooser.setDefaultOption("Drive to Tag", m_driveToTag);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //Spins Motor if April Tags are Recognized for 20 Ticks

        // new JoystickButton(m_driverController, XboxController.Button.kB.value).
        // onTrue(new InstantCommand(()-> m_armSubsystem.spinMotor(.1))).
        // onFalse(new InstantCommand(()-> m_armSubsystem.spinMotor(0)));

        new JoystickButton(m_driverController, XboxController.Button.kStart.value).
        onTrue(new InstantCommand(()-> m_armSubsystem.spinTelescopingMotor(0.3)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.stopTelescopingMotor()));

        new JoystickButton(m_driverController, XboxController.Button.kBack.value).
        onTrue(new InstantCommand(()-> m_armSubsystem.spinTelescopingMotor(-0.3)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.stopTelescopingMotor()));

        //This runs Endeffector to Collect Cube
        new JoystickButton(m_driverController, XboxController.Button.kA.value).
        onTrue(new InstantCommand(()-> m_armSubsystem.spinEndEffector(0.5)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.spinEndEffector(0.07)));

        //This runs Endeffector to Collect Cone
        new JoystickButton(m_driverController, XboxController.Button.kB.value).
        onTrue(new InstantCommand(()-> m_armSubsystem.spinEndEffector(0.5)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.stopEndEffector()));

        new JoystickButton(m_driverController, XboxController.Button.kX.value).
        onTrue(new InstantCommand(()-> m_armSubsystem.spinRotationMotors(0.2)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.stopRotationMotors()));

        new JoystickButton(m_driverController, XboxController.Button.kY.value).
        onTrue(new InstantCommand(()-> m_armSubsystem.spinRotationMotors(0.5)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.stopRotationMotors()));
        //This runs Endeffector to eject game peices
        driverRightTrigger.
        onTrue(new InstantCommand(()-> m_armSubsystem.spinEndEffector(-0.2)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.stopEndEffector()));
  
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   public void disablePIDSubsystems() {
    // m_armSubsystem.disable();
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    return m_chooser.getSelected();
  }
}
