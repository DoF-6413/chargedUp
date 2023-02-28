// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.SimulationDevices.SparkMaxWrapper;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private static CANSparkMax leftLead;
  private static CANSparkMax rightLead;
  private static CANSparkMax leftFollower1;
  private static CANSparkMax rightFollower1;
  private static CANSparkMax leftFollower2;
  private static CANSparkMax rightFollower2;

  private static RelativeEncoder encoderLeftLead;
  private static RelativeEncoder encoderRightLead;

  private static EncoderSim m_rightSimEncoder;
  private static EncoderSim m_leftSimEncoder;

  private static DifferentialDrive diffDrive;

  public DifferentialDrivetrainSim m_drivetrainSimulator;

  public static DifferentialDriveKinematics m_Kinematics;
  public static DifferentialDriveWheelSpeeds m_WheelSpeeds;

  private static DifferentialDriveOdometry m_odometry;
  public static Field2d m_field2d;

  private static Encoder realEncoderRightRep;
  private static Encoder realEncoderLeftRep;

  private static GyroSubsystem gyro = new GyroSubsystem();
  SimDouble gyroAngleSim;

  public DrivetrainSubsystem() {

    leftLead = new SparkMaxWrapper(DrivetrainConstants.kDrivetrainCANIDs[3], MotorType.kBrushless);
    rightLead = new SparkMaxWrapper(DrivetrainConstants.kDrivetrainCANIDs[0], MotorType.kBrushless);

    leftLead.setInverted(DrivetrainConstants.kLeftInverted);
    rightLead.setInverted(DrivetrainConstants.kRightInverted);

    leftFollower1 = new SparkMaxWrapper(DrivetrainConstants.kDrivetrainCANIDs[4], MotorType.kBrushless);
    rightFollower1 = new SparkMaxWrapper(DrivetrainConstants.kDrivetrainCANIDs[1], MotorType.kBrushless);

    leftFollower2 = new SparkMaxWrapper(DrivetrainConstants.kDrivetrainCANIDs[5], MotorType.kBrushless);
    rightFollower2 = new SparkMaxWrapper(DrivetrainConstants.kDrivetrainCANIDs[2], MotorType.kBrushless);

    Arrays.asList(leftLead, leftFollower1, leftFollower2, rightLead, rightFollower1, rightFollower2)
        .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

    realEncoderLeftRep = new Encoder(4, 5);
    realEncoderRightRep = new Encoder(2, 3);

    realEncoderLeftRep.setDistancePerPulse(0.0238095238);
    realEncoderRightRep.setDistancePerPulse(0.0238095238);

    realEncoderLeftRep.setReverseDirection(true);

    encoderLeftLead = leftLead.getEncoder();
    encoderRightLead = rightLead.getEncoder();

    leftFollower1.follow(leftLead);
    leftFollower2.follow(leftLead);

    encoderLeftLead.setPositionConversionFactor(DrivetrainConstants.kTicksToFeat);
    encoderRightLead.setPositionConversionFactor(DrivetrainConstants.kTicksToFeat);

    rightFollower1.follow(rightLead);
    rightFollower2.follow(rightLead);

    diffDrive = new DifferentialDrive(leftLead, rightLead);
    diffDrive.setSafetyEnabled(false);

    m_odometry = new DifferentialDriveOdometry(
        gyro.getRotation2d(), getPositionLeftLead(), getPositionRightLead());

    m_Kinematics = new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidth);

    if (RobotBase.isSimulation()) {
      m_drivetrainSimulator = new DifferentialDrivetrainSim(
          // todo: will fix :)
          DCMotor.getNEO(3),
          DrivetrainConstants.kgearing,
          3,
          DrivetrainConstants.kMass,
          DrivetrainConstants.kwheelRadiusMeters,
          DrivetrainConstants.kTrackWidth,
          // The standard deviations for measurement noise:
          // x and y: 0.001 m
          // heading: 0.001 rad
          // l and r velocity: 0.1 m/s
          // l and r position: 0.005 m
          VecBuilder.fill(5, 5, 0.001, 0.1, 0.1, 0.005, 0.005));

      m_leftSimEncoder = new EncoderSim(realEncoderLeftRep);
      m_rightSimEncoder = new EncoderSim(realEncoderRightRep);

      gyroAngleSim = new SimDeviceSim("AHRS[" + SPI.Port.kMXP.value + "]").getDouble("Angle");

      m_field2d = new Field2d();

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    if (m_field2d != null
    // && m_odometry.getPoseMeters() != null
    ) {
      setRobotFromFieldPose();
    }
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putString("Pose", getPose().toString());
  }

  public void SmartDashboardCalls() {
    SmartDashboard.putNumber("Drivetrain Position", this.getPositionRightLead());

  }

  public void setRaw(double driveValue, double turnValue) {
    // if(Robot.isReal()){
    diffDrive.arcadeDrive(driveValue, turnValue);
    // }
    // else if(Robot.isSimulation()){
    // m_SimSub.setVoltage(driveValue, turnValue);
    // }
    SmartDashboard.putNumber("Drive Value", driveValue);
    SmartDashboard.putNumber("Turn Value", turnValue);
    SmartDashboard.putNumber("Left Lead", leftLead.get());
    SmartDashboard.putNumber("Right Lead", rightLead.get());

  }

  public CANSparkMax getLeftMotor() {
    return leftLead;
  }

  public CANSparkMax getRightMotor() {
    return rightLead;
  }

  public double getPositionLeftLead() {
    if (RobotBase.isSimulation()) {
      return realEncoderLeftRep.getDistance();
    } else {
      return encoderLeftLead.getPosition();
    }
  }

  public double getPositionRightLead() {
    if (RobotBase.isSimulation()) {
      return realEncoderRightRep.getDistance();
    } else {
      return encoderRightLead.getPosition();
    }
  }

  public void resetPosition() {
    encoderLeftLead.setPosition(0);
    encoderRightLead.setPosition(0);
  }

  public void setRobotFromFieldPose() {
    if (RobotBase.isSimulation()) {
      setPose(m_odometry.getPoseMeters());
    }
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    if (RobotBase.isSimulation()) {
      // This is a bit hokey, but if the Robot jumps on the field, we need
      // to reset the internal state of the DriveTrainSimulator.
      // No method to do it, but we can reset the state variables.
      // NOTE: this assumes the robot is not moving, since we are not resetting
      // the rate variables.
      // m_drivetrainSimulator.setState(new Matrix<>(Nat.N7(), Nat.N1()));

      // reset the GyroSim to match the driveTrainSim
      // do it early so that "real" odometry matches this value
      gyroAngleSim.set(m_drivetrainSimulator.getHeading().getDegrees());
      m_field2d.setRobotPose(pose);
    }

    // realEncoderLeftRep.reset();
    // realEncoderRightRep.reset();
    // m_odometry.resetPosition( Rotation2d.fromDegrees(gyro.getAngle()),
    // realEncoderLeftRep.getDistance(), realEncoderRightRep.getDistance(), pose);
  }

  public void updateOdometry() {
    if (RobotBase.isSimulation()) {
      m_odometry.update(gyro.getRotation2d(), getPositionRightLead(), getPositionLeftLead());
      SmartDashboard.putNumber("Left position", getPositionLeftLead());
      SmartDashboard.putNumber("Right position", getPositionRightLead());
    } else {
      m_odometry.update(gyro.getRotation2d(), getPositionRightLead(), getPositionLeftLead());
    }
  }

  public void resetOdometry(Pose2d currentPose2d) {
    m_odometry.resetPosition(gyro.getRotation2d(), getPositionRightLead(), getPositionLeftLead(), currentPose2d);
    
  }

  public double getHeading() {
    return m_odometry.getPoseMeters().getRotation().getDegrees();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(realEncoderLeftRep.getRate(), realEncoderRightRep.getRate());
  }

  @Override
  public void simulationPeriodic() {
    m_drivetrainSimulator.setInputs(
        (rightLead.get() * RobotController.getBatteryVoltage()),
        leftLead.get() * RobotController.getBatteryVoltage());

    m_drivetrainSimulator.update(0.020);

    m_leftSimEncoder.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftSimEncoder.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());

    m_rightSimEncoder.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightSimEncoder.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

    gyroAngleSim.set(m_drivetrainSimulator.getHeading().getDegrees());
  }

}
