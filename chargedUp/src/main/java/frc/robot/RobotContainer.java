// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TelescoperConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.GyroSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.VisionConstants;
// import frc.robot.commands.targetFinding;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.commands.ArmPID;
// import frc.robot.commands.targetFinding;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.commands.ArmPID;
import frc.robot.commands.ArmControls.RotationPID;
import frc.robot.commands.ArmControls.RotationReset;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.colorSensor;
// import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.colorSensor;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.commands.getEstimatedPose;
// import frc.robot.commands.getPoseAprilTag;
// import frc.robot.commands.ArmControls.TelescoperConditional;
import frc.robot.commands.ArmControls.TelescoperPID;
import frc.robot.commands.ArmControls.TelescoperReset;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.TelescoperSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// import frc.robot.subsystems.colorSensor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.commands.ArmControls.EndEffectorRunner;
import frc.robot.commands.ArmControls.RotationPID;
// import frc.robot.commands.ArmControls.TelescoperConditional;
import frc.robot.commands.ArmControls.TelescoperPID;
import frc.robot.commands.ArmControls.TelescoperReset;
import frc.robot.commands.Autos.BackingOutArm;
import frc.robot.commands.Autos.CenterLScoreOutBalance;
import frc.robot.commands.Autos.CenterRScoreOutBalance;
import frc.robot.commands.Autos.G1TRAroundCSBalance;
import frc.robot.commands.Autos.G3TLAroundCSBalance;
import frc.robot.commands.Autos.ScoreCone;
import frc.robot.commands.Autos.ScoreRunRight;
import frc.robot.commands.Autos.scoreRun;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;
import frc.robot.subsystems.WristSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.colorSensor;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem();
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(m_gyroSubsystem);
  
  private final TelescoperSubsystem m_telescoperSubsystem = new TelescoperSubsystem();
  private final EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();
  private final PoseEstimator m_PoseEstimator = new PoseEstimator(m_gyroSubsystem, m_drivetrainSubsystem, m_visionSubsystem,DrivetrainConstants.kinematics );
 

  private final WristSubsystem m_wristSubsystem = new WristSubsystem();

  //warning means not used, but its here so it calls the periodic for the subsystem DO NOT REMOVE
  // private final colorSensor m_colorSensorSubsystem = new colorSensor();

  PathPlannerTrajectory testPath = PathPlanner.loadPath("TestPath", new PathConstraints(2, 0.8));
  PathPlannerTrajectory newishPath = PathPlanner.loadPath("VisionTest", new PathConstraints(2, 0.8));
  PathPlannerTrajectory getOntoChargingStation = PathPlanner.loadPath("GetOntoCSJanky", new PathConstraints(2, 0.8));
 PathPlannerTrajectory RightRed2 = PathPlanner.loadPath("RightRed2", new PathConstraints(4, 4));
  // PathPlannerTrajectory testPath = PathPlanner.loadPath("TestPath", new PathConstraints(2, 0.8));
  PathPlannerTrajectory runOutCommunity = PathPlanner.loadPath("OutCommunity", new PathConstraints(1.5, 0.8));
  PathPlannerTrajectory overCSBalance = PathPlanner.loadPath("OverCSBalance", new PathConstraints(1.5, 1.2));
  


 PathPlannerTrajectory RightRed2Traj = PathPlanner.generatePath(
  new PathConstraints(0.5,0.5), 
  new PathPoint(m_drivetrainSubsystem.getPose().getTranslation(),m_gyroSubsystem.getRotation2d()),
new PathPoint(RightRed2.getInitialPose().getTranslation(),RightRed2.getInitialPose().getRotation())
);

  PathPlannerTrajectory traj1 = PathPlanner.generatePath(
    new PathConstraints(0.25, 0.25), 
    new PathPoint(m_drivetrainSubsystem.getPose().getTranslation(),m_gyroSubsystem.getRotation2d()), // position, heading
    new PathPoint(new Translation2d(9.178784,6.749796), new Rotation2d(0)) // position, heading
);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      public final static CommandXboxController m_auxController =
      new CommandXboxController(OperatorConstants.kAuxControllerPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SendableChooser<Command> m_chooser = new SendableChooser<>();
  public RobotContainer() {
    // Configure the trigger bindings
    // m_chooser.setDefaultOption("Test Path", new TrajectoryRunner(m_drivetrainSubsystem, testPath.relativeTo(m_drivetrainSubsystem.getPose()), true));
    m_chooser.addOption("Score Grid1 TL Run", new scoreRun(m_armSubsystem, m_drivetrainSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem));
    // m_chooser.addOption("Score Grid1 TR Around CS Balance", new G1TRAroundCSBalance(m_armSubsystem, m_drivetrainSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_gyroSubsystem));
    m_chooser.addOption("Score Grid2 TL Over CS Balance", new CenterLScoreOutBalance(m_armSubsystem, m_drivetrainSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_gyroSubsystem));
    // m_chooser.addOption("Score Grid2 TR Over CS Balance", new CenterRScoreOutBalance(m_armSubsystem, m_drivetrainSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_gyroSubsystem));
    // m_chooser.addOption("Score Grid3 TL Around CS Balance", new G3TLAroundCSBalance(m_armSubsystem, m_drivetrainSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_gyroSubsystem));
    // m_chooser.addOption("Score Grid3 TR Run", new ScoreRunRight(m_armSubsystem, m_drivetrainSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem));
    // m_chooser.addOption("Score Grid1 TL Run Follow Path With Events", new ScoreRunFollowWithEvents(m_armSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_drivetrainSubsystem));
    m_chooser.addOption("Out of Community", new TrajectoryRunner(m_drivetrainSubsystem, runOutCommunity.relativeTo(m_drivetrainSubsystem.getPose()), true));
    m_chooser.addOption("Out of Community and Balance", new TrajectoryRunner(m_drivetrainSubsystem, overCSBalance.relativeTo(m_drivetrainSubsystem.getPose()), true));
    SmartDashboard.putData("m_chooser", m_chooser);
    configureBindings();
    defaultCommands();
      
  }

  public void defaultCommands(){
    m_drivetrainSubsystem.setDefaultCommand(new RunCommand(() ->
     m_drivetrainSubsystem.setRaw(-m_driverController.getLeftY(), -m_driverController.getRightX()), m_drivetrainSubsystem));

     m_armSubsystem.setDefaultCommand(new RunCommand(() -> m_armSubsystem.spinRotationMotors(-m_auxController.getLeftY()), m_armSubsystem));
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
        m_auxController.start().
        onTrue(new InstantCommand(()-> m_telescoperSubsystem.spinTelescopingMotor(0.8), m_telescoperSubsystem))
        .onFalse(new InstantCommand(()-> m_telescoperSubsystem.stopTelescopingMotor(), m_telescoperSubsystem));

        m_auxController.back().
        onTrue(new InstantCommand(()-> m_telescoperSubsystem.spinTelescopingMotor(-0.8), m_telescoperSubsystem))
        .onFalse(new InstantCommand(()-> m_telescoperSubsystem.stopTelescopingMotor(), m_telescoperSubsystem));

        // //This runs Endeffector to Collect Cube
        // m_auxController.b().
        // onTrue(new InstantCommand(()-> m_armSubsystem.spinEndEffector(0.5)))
        // .onFalse(new InstantCommand(()-> m_armSubsystem.spinEndEffector(0.07)));

        // //This runs Endeffector to Collect Cone
        m_auxController.a().
        onTrue(new InstantCommand(()-> m_armSubsystem.resetRotationPosition()));
        
        
        
        // // This runs Endeffector to eject game peices
        // m_auxController.rightTrigger().
        // onTrue(new InstantCommand(()-> m_armSubsystem.spinEndEffector(-0.2)))
        // .onFalse(new InstantCommand(()-> m_armSubsystem.stopEndEffector()));
  
        m_auxController.y()
        .onTrue(
          new ConditionalCommand(
          new TelescoperPID(m_telescoperSubsystem, 0), 
          new TelescoperPID(m_telescoperSubsystem, 130), 
          () ->  m_armSubsystem.isInFramePerimeter()
          ))
        .onFalse(
          new ConditionalCommand(
          new TelescoperPID(m_telescoperSubsystem, 0), 
          new TelescoperPID(m_telescoperSubsystem, 0), 
          () -> m_armSubsystem.isInFramePerimeter()
          ));

        // m_auxController.y()
        // .onTrue(
        //   new ConditionalCommand(
        //   new InstantCommand(() -> System.out.println("Pressed Y and is True")), 
        //   new InstantCommand(() -> System.out.println("Pressed Y and is False")), 
        //   () ->  m_armSubsystem.isInFramePerimeter()
        //   ))
        // .onFalse(
        //   new ConditionalCommand(
        //   new InstantCommand(() -> System.out.println("Released Y and is True")), 
        //   new InstantCommand(() -> System.out.println("Released Y and is False")), 
        //   () -> m_armSubsystem.isInFramePerimeter()
        //   ));

        m_auxController.x().
        onTrue(new TelescoperPID(m_telescoperSubsystem, 50));

        m_auxController.rightBumper().onTrue(new TelescoperReset(m_telescoperSubsystem));

        m_auxController.leftTrigger().
        onTrue(new InstantCommand(()-> SmartDashboard.putNumber("Best fiducialID", m_visionSubsystem.getBestFiducialID())));
        // .onFalse(new InstantCommand(()-> SmartDashboard.putNumber("Best fiducialID", 0)));

        // m_auxController.leftBumper().onTrue(new getPoseAprilTag(m_drivetrainSubsystem, m_visionSubsystem, m_gyroSubsystem));

        m_driverController.a().onTrue( new getEstimatedPose(m_gyroSubsystem, m_drivetrainSubsystem, m_visionSubsystem, m_PoseEstimator));

        m_driverController.y().onTrue(new TrajectoryRunner(m_drivetrainSubsystem, traj1, true));
        //This extends telescoper 
        m_auxController.start().
        onTrue(new InstantCommand(()-> m_armSubsystem.spinTelescopingMotor(0.8)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.stopTelescopingMotor()));

        //This unextends telescoper 
        m_auxController.back().
        onTrue(new InstantCommand(()-> m_armSubsystem.spinTelescopingMotor(-0.8)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.stopTelescopingMotor()));

        //This runs Endeffector to Collect Cube
        m_auxController.b().
        onTrue(new InstantCommand(()-> m_armSubsystem.spinEndEffector(0.5)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.spinEndEffector(0.07)));

        m_auxController.back().onTrue(new TelescoperReset(m_telescoperSubsystem));
        m_auxController.start().onTrue(new InstantCommand(()-> m_armSubsystem.resetRotationPosition()));

        m_auxController.leftBumper().
           onTrue(new InstantCommand(()-> m_endEffectorSubsystem.spinEndEffector(0.5)))
          .onFalse(new InstantCommand(()-> m_endEffectorSubsystem.stopEndEffector()));
        
        m_auxController.rightBumper().
           onTrue(new InstantCommand(()-> m_endEffectorSubsystem.spinEndEffector(0.5)))
          .onFalse(new InstantCommand(()-> m_endEffectorSubsystem.spinEndEffector(0.07)));

        // // This runs Endeffector to eject game peices
        m_auxController.rightTrigger().
           onTrue(new InstantCommand(()-> m_endEffectorSubsystem.spinEndEffector(-0.5)))
          .onFalse(new InstantCommand(()-> m_endEffectorSubsystem.stopEndEffector()));

        m_auxController.y()
            .onTrue(
              // new ConditionalCommand(
              // new TelescoperPID(m_telescoperSubsystem, 0), 
              new TelescoperPID(m_telescoperSubsystem, TelescoperConstants.kMaxExtention))
              // () ->  m_armSubsystem.isInFramePerimeter()
              // ))
            .onFalse(new TelescoperPID(m_telescoperSubsystem, 0));


        m_auxController.x().onTrue(new InstantCommand(()-> m_telescoperSubsystem.spinTelescopingMotor(1)))
        .onFalse(new InstantCommand(()-> m_telescoperSubsystem.stopTelescopingMotor()));

        m_auxController.a().onTrue(new InstantCommand(()-> m_wristSubsystem.spinWrist(.70)))
        .onFalse(new InstantCommand(()-> m_wristSubsystem.stopWrist()));

        m_auxController.b().onTrue(new InstantCommand(()-> m_wristSubsystem.spinWrist(-.70)))
        .onFalse(new InstantCommand(()-> m_wristSubsystem.stopWrist()));

    
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    return m_chooser.getSelected();
  }
}
