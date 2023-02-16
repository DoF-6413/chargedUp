// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.Instant;
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
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.NotifyCallback;
import edu.wpi.first.hal.simulation.SimValueCallback;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.SimulationDevices.SparkMaxWrapper;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

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
  public static  DifferentialDriveWheelSpeeds m_WheelSpeeds;

private static DifferentialDriveOdometry m_odometry;
public static Field2d m_field2d;
  
private static Encoder simEncoderRightLead;
private static Encoder simEncoderLeftLead;

private static EncoderSim simEncoderRight;
private static EncoderSim simEncoderLeft;
private static GyroSubsystem gyro = new GyroSubsystem();
SimDouble gyroAngleSim;


  public DrivetrainSubsystem() {

    leftLead = new SparkMaxWrapper(DrivetrainConstants.kDrivetrainCANIDs[3], MotorType.kBrushless);
    rightLead = new SparkMaxWrapper(DrivetrainConstants.kDrivetrainCANIDs[0], MotorType.kBrushless);

    leftFollower1 = new SparkMaxWrapper(DrivetrainConstants.kDrivetrainCANIDs[4], MotorType.kBrushless);
    rightFollower1 = new SparkMaxWrapper(DrivetrainConstants.kDrivetrainCANIDs[1], MotorType.kBrushless);

    leftFollower2 = new SparkMaxWrapper(DrivetrainConstants.kDrivetrainCANIDs[5], MotorType.kBrushless);
    rightFollower2 = new SparkMaxWrapper(DrivetrainConstants.kDrivetrainCANIDs[2], MotorType.kBrushless);

    Arrays.asList(leftLead, leftFollower1, leftFollower2, rightLead, rightFollower1, rightFollower2)
                .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));
    
    simEncoderLeftLead = new Encoder(4, 5);
    simEncoderRightLead = new Encoder(2, 3);
simEncoderLeftLead.setDistancePerPulse(0.00155852448);
simEncoderRightLead.setDistancePerPulse(0.00155852448);
    encoderLeftLead = leftLead.getEncoder();
    encoderRightLead = rightLead.getEncoder();
    
    leftLead.setInverted(DrivetrainConstants.kLeftInverted);
    // todo: uncomment for conversion
    encoderLeftLead.setPositionConversionFactor(DrivetrainConstants.kTicksToFeat);
    // encoderLeftLead.setInverted(DrivetrainConstants.kLeftInverted);

    leftFollower1.follow(leftLead);
    leftFollower2.follow(leftLead);


    encoderRightLead = rightLead.getEncoder();

    // todo: uncomment for conversion
    // encoderRightLead.setPositionConversionFactor(DrivetrainConstants.kTicksToFeat);
    rightLead.setInverted(DrivetrainConstants.kRightInverted);
    
    rightFollower1.follow(rightLead);
    rightFollower2.follow(rightLead);
    
    
    diffDrive = new DifferentialDrive(leftLead, rightLead);
    
    m_odometry = new DifferentialDriveOdometry(
      gyro.getRotation2d(), DrivetrainSubsystem.getDistanceLeaftlead(), DrivetrainSubsystem.getDistanceRigthlead());
      
       if(RobotBase.isSimulation()){
        m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
            // todo: will fix :)
              DCMotor.getNEO(3),
              DrivetrainConstants.kgearing,
              3,
              DrivetrainConstants.kMass,
              DrivetrainConstants.kwheelRadiusMeters,
              DrivetrainConstants.ktrackWidth,
              // The standard deviations for measurement noise:
              // x and y:          0.001 m
              // heading:          0.001 rad
              // l and r velocity: 0.1   m/s
              // l and r position: 0.005 m
              VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

              m_leftSimEncoder = new EncoderSim(simEncoderLeftLead);
              m_rightSimEncoder = new EncoderSim(simEncoderRightLead);

              gyroAngleSim = new SimDeviceSim("AHRS[" + SPI.Port.kMXP.value +"]").getDouble("Angle");

              m_field2d = new Field2d();
       }
  }

  public static Pose2d getPose () {
    return m_odometry.getPoseMeters();
}

public void setPose(Pose2d pose) {
    if (RobotBase.isSimulation()) {
        // This is a bit hokey, but if the Robot jumps on the field, we need
        //   to reset the internal state of the DriveTrainSimulator.
        //   No method to do it, but we can reset the state variables.
        //   NOTE: this assumes the robot is not moving, since we are not resetting
        //   the rate variables.
        m_drivetrainSimulator.setState(new Matrix<>(Nat.N7(), Nat.N1()));

        // reset the GyroSim to match the driveTrainSim
        // do it early so that "real" odometry matches this value
        gyroAngleSim.set(-m_drivetrainSimulator.getHeading().getDegrees());
        m_field2d.setRobotPose(pose);
    }

    // simEncoderLeftLead.reset();
    // simEncoderRightLead.reset();
    // m_odometry.resetPosition( Rotation2d.fromDegrees(gyro.getAngle()), simEncoderLeftLead.getDistance(), simEncoderRightLead.getDistance(), pose);
    }
  

  public void setRaw(double driveValue, double turnValue){
  //   if(Robot.isReal()){
    diffDrive.arcadeDrive(driveValue, turnValue);
  // }
  //   else if(Robot.isSimulation()){
  //   m_SimSub.setVoltage(driveValue, turnValue);
  // }
    SmartDashboard.putNumber("Drive Value", driveValue);
    SmartDashboard.putNumber("Turn Value", turnValue);
    SmartDashboard.putNumber("Left Lead", leftLead.get());
    SmartDashboard.putNumber("Right Lead", rightLead.get());
    }

  public double getPositionLeftLead(){
    return encoderLeftLead.getPosition();
    
  }

  public double getPositionRightLead(){
    return encoderRightLead.getPosition();
  }

  public void resetPosition(){
    encoderLeftLead.setPosition(0);
  }
// use this later to set to specific pose by passing in an argument to this void
//   public void resetPose(){
// m_odometry.resetPosition(null, getDistance(), getDistance(), null);
//   }
  
  public static double getDistanceLeaftlead(){
    return encoderLeftLead.getPositionConversionFactor();

  }
  public static double getDistanceRigthlead(){
    return  encoderRightLead.getPositionConversionFactor();

  }


  public static void updateOdometry(){
    m_odometry.update(gyro.getRotation2d(), getDistanceRigthlead(), getDistanceLeaftlead());
  }
 public static void resetOdometry(Pose2d currentPose2d){
  m_odometry.resetPosition(gyro.getRotation2d(), getDistanceRigthlead(), getDistanceLeaftlead(), currentPose2d);
 }
  public static double getHeading(){
    return m_odometry.getPoseMeters().getRotation().getDegrees();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("is running periodic");
    updateOdometry();
    m_field2d.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putString("Pose", getPose().toString());
 }
  

  public void SmartDashboardCalls(){
    SmartDashboard.putNumber("Drivetrain Position", this.getPositionRightLead());

  }
  
 public DifferentialDriveWheelSpeeds getWheelSpeeds(){
  return new DifferentialDriveWheelSpeeds(simEncoderLeftLead.getRate(), simEncoderRightLead.getRate());
 }

    public CANSparkMax getLeftMotor(){
      return leftLead;
    }

    public CANSparkMax getRightMotor(){
      return rightLead;
    }


 

  public void simulationPeriodic() {
    m_drivetrainSimulator.setInputs(
      (-leftLead.get() * RobotController.getBatteryVoltage()),
        rightLead.get() * RobotController.getBatteryVoltage());
      
        System.out.println("is running sim");
  m_drivetrainSimulator.update(0.020);

   m_leftSimEncoder.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftSimEncoder.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
  
  m_rightSimEncoder.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightSimEncoder.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
      
    gyroAngleSim.set(-m_drivetrainSimulator.getHeading().getDegrees());
    
    m_field2d.setRobotPose(getPose());
  }

  public void setRobotFromFieldPose(){
    if(RobotBase.isSimulation()){
      setPose(m_field2d.getRobotPose());
    }
  }

 public Command RamseteController(PathPlannerTrajectory Traj, boolean firstPath){
  return new SequentialCommandGroup(
    new InstantCommand(()-> {
    if (firstPath == true) {
      DrivetrainSubsystem.resetOdometry(Traj.getInitialPose());
    }
  }
  ),
  new PPRamseteCommand(
    Traj, 
    poseSupplier(),
    new RamseteController(), 
    // new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV, DrivetrainConstants.kA), 
    DrivetrainSubsystem.m_Kinematics,
    getVoltage(),
    // DrivetrainSubsystem.m_WheelSpeeds,
      this)
  );
 }

private Supplier<Pose2d> poseSupplier() {
return () -> getPose();
}


private BiConsumer<Double, Double> getVoltage() {
  //This is taking two inputs and printing the combination of the two in a particulat method
  BiConsumer<Double, Double> voltage = (a, b) -> System.out.println((a + b)/2);
 return voltage;
}

//  
}
