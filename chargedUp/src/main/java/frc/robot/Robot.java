// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;
import com.pathplanner.lib.server.PathPlannerServerThread;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Timer m_timer;
  private RobotContainer m_robotContainer;
  
  private final RamseteController m_ramseteController = new RamseteController();
  private Trajectory m_Trajectory;


  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensorV3 = new ColorSensorV3 (i2cPort);
  //when plug into roborio check the port because might need to change port 

  private final ColorMatch m_colorMatcher = new ColorMatch();


  private final Color kpurple = new Color(0.2502, 0.2502, 0.5002);
  private final Color kyellow = new Color(0.5315, 0.4438, 0.02515);
    /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    PathPlannerServer.startServer(5811);
    // PathPlannerServer.startServer(5811);
    // PathPlannerTrajectory firstPath = PathPlanner.loadPath("firstPath", null);
    // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.
  

        // DrivetrainSubsystem.m_field2d.getObject("firstPath").setTrajectory(m_Trajectory);
        // SmartDashboard.putString("FieldSTUFFF","HOLAAAA MUNDO");
    // Push the trajectory to Field2d.

    m_colorMatcher.addColorMatch(kpurple);
    m_colorMatcher.addColorMatch(kyellow);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    Color detectedColor = m_colorSensorV3.getColor();

    String colorString; 
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kpurple){
      colorString = "cube";
      System.out.println("cube");
    } else if (match.color == kyellow){
      colorString = "cone";
      System.out.println("cone");
    }


    double IR = m_colorSensorV3.getIR();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    

    int proximity = m_colorSensorV3.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // if(RobotBase.isSimulation()){
    //   RobotContainer.getDrive().setRobotFromFieldPose();
    // }
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
    m_timer = new Timer();
    m_timer.start();

    // Reset the drivetrain's odometry to the starting pose of the trajectory.
    RobotContainer.m_drivetrainSubsystem.resetOdometry(m_Trajectory.getInitialPose());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    RobotContainer.m_drivetrainSubsystem.updateOdometry();

    // // Update robot position on Field2d.
    RobotContainer.m_drivetrainSubsystem.setRobotFromFieldPose();

    if (m_timer.get() < m_Trajectory.getTotalTimeSeconds()) {
      // Get the desired pose from the trajectory.
      var desiredPose = m_Trajectory.sample(m_timer.get());

      // Get the reference chassis speeds from the Ramsete controller.
      var refChassisSpeeds = m_ramseteController.calculate(RobotContainer.m_drivetrainSubsystem.getPose(), desiredPose);

      // Set the linear and angular speeds.
      RobotContainer.m_drivetrainSubsystem.setRaw(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
    } else {
      RobotContainer.m_drivetrainSubsystem.setRaw(0, 0);
    }
  }
  
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (RobotBase.isSimulation()) {
    //   DrivetrainSubsystem.setRobotFromFieldPose()
    // }
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    RobotContainer.m_drivetrainSubsystem.updateOdometry();
    RobotContainer.m_drivetrainSubsystem.setRobotFromFieldPose();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
 
 
}
