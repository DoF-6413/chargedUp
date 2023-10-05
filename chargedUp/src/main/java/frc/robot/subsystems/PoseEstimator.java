// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class PoseEstimator extends SubsystemBase {
  
  
  private OriginPosition originPosition = kRedAllianceWallRightSide;
  
  /**
   * Standard deviations of model states. Increase these numbers to trust your
   * model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
   * meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
   * radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

  private double previousPipelineTimestamp = 0;
  
  VisionSubsystem m_visionSubsystem;
  private static DifferentialDrivePoseEstimator poseEstimator;
  static GyroSubsystem m_gyroSubsystem;
  static DrivetrainSubsystem m_drivetrainSubsystem;
  // Field2d field2d = new Field2d();
  Pose3d visionMeasurement;
  public double resultsTimestamp;
  static SimDouble gyroAngleSim;
  public static DifferentialDrivetrainSim m_drivetrainSimulator;
  public static Field2d m_field2d;
  public PhotonPipelineResult PhotonPipelineResult;

  /** Creates a new PoseEstimator. */
  public PoseEstimator(GyroSubsystem gyro, DrivetrainSubsystem drive, VisionSubsystem vision,
  DifferentialDriveKinematics kinematics) {

    m_field2d = new Field2d();
    SmartDashboard.putData(m_field2d);

    m_drivetrainSubsystem = drive;
    m_gyroSubsystem = gyro;
    m_visionSubsystem = vision;
    setAlliance(DriverStation.getAlliance());
    poseEstimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), drive.getPositionLeftLead(),
        drive.getPositionRightLead(), new Pose2d(), stateStdDevs, visionMeasurementStdDevs);

  }

  private static Pose2d flipAlliance(Pose2d pose2d) {
    return flipAlliance(pose2d).relativeTo(VisionConstants.FLIPPING_POSE);
  }

  public void setAlliance(Alliance alliance) {
    boolean allianceChanged = false;
    switch (alliance) {
      case Blue:
        allianceChanged = (originPosition == kRedAllianceWallRightSide);
        originPosition = kBlueAllianceWallRightSide;
        break;
      case Red:
        allianceChanged = (originPosition == kBlueAllianceWallRightSide);
        originPosition = kRedAllianceWallRightSide;
        break;
      default:
        // No valid alliance data. Nothing we can do about it sawTag
        if (allianceChanged && m_visionSubsystem.seeTarget() && VisionConstants.useAprilTag) {

          // The alliance changed, which changes the coordinate system.
          // Since a tag was seen, and the tags are all relative to the coordinate system,
          // the estimated pose
          // needs to be transformed to the new coordinate system.
          var newPose = flipAlliance(getcurrentPose());
          poseEstimator.resetPosition(m_gyroSubsystem.getRotation2d(), m_drivetrainSubsystem.getPositionLeftLead(),
              m_drivetrainSubsystem.getPositionRightLead(), newPose);

            }
            //Code is dead bc useAprilTag is set to false
          }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
    if (DrivetrainSubsystem.m_field2d != null) {
      this.setRobotFromFieldPose();
    } 
    m_field2d.setRobotPose(getcurrentPose());
    

    poseEstimator.updateWithTime(Timer.getFPGATimestamp(),
        m_gyroSubsystem.getRotation2d(), m_drivetrainSubsystem.getPositionLeftLead(),
        m_drivetrainSubsystem.getPositionRightLead());

    PhotonPipelineResult = VisionSubsystem.camera.getLatestResult();
    resultsTimestamp = PhotonPipelineResult.getTimestampSeconds();
SmartDashboard.putNumber("photonTime", PhotonPipelineResult.getTimestampSeconds());
SmartDashboard.putNumber("FPGA TIme", Timer.getFPGATimestamp());
    if (resultsTimestamp != previousPipelineTimestamp && PhotonPipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultsTimestamp;
      var target = PhotonPipelineResult.getBestTarget(); 
      if (VisionConstants.useAprilTag = true){
      var fiducialid = target.getFiducialId();
      
      if (target.getPoseAmbiguity() <= 0.2 && fiducialid >= 0 && fiducialid < 9) {

        AprilTagFieldLayout atfl;
        try {
          atfl = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField().getTags(), 16.4592,
              8.2296);
          Pose3d targetPose = atfl.getTagPose(fiducialid).get();
          Transform3d camToTarget = target.getBestCameraToTarget();
          Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
          visionMeasurement = camPose.transformBy(VisionConstants.cameraOnRobot);
          SmartDashboard.putString("visionmeasure", visionMeasurement.toPose2d().toString());
          SmartDashboard.putNumber("timestamp", resultsTimestamp);
          PoseEstimator.poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), Timer.getFPGATimestamp(),
              visionMeasurementStdDevs);
        
        } catch (IOException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();

        }}
      }
      
    }

    
    
    SmartDashboard.putString("final Pose",
    PoseEstimator.poseEstimator.getEstimatedPosition().toString());
  }

  public Pose2d getcurrentPose() {

    return poseEstimator.getEstimatedPosition();
  }

  public static void resetPose(Pose2d currentPose2d) {
    poseEstimator.resetPosition(m_gyroSubsystem.getRotation2d(), m_drivetrainSubsystem.getPositionRightLead(),
        m_drivetrainSubsystem.getPositionLeftLead(), currentPose2d);
  }

  public void setRobotFromFieldPose() {
    if (RobotBase.isSimulation()) {
      setPose(poseEstimator.getEstimatedPosition());
    }
    
  }

  public static void setPose(Pose2d pose) {
    if (RobotBase.isSimulation()) {
      // reset the GyroSim to match the driveTrainSim
      // do it early so that "real" 0dometry matches this value
      m_drivetrainSubsystem.gyroAngleSim.set(m_drivetrainSubsystem.m_drivetrainSimulator.getHeading().getDegrees());
      DrivetrainSubsystem.m_field2d.setRobotPose(pose);
    }
  }
}
