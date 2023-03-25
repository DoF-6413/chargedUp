// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.GyroSubsystem;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;



public class PoseEstimator extends SubsystemBase {

  private OriginPosition originPosition = kBlueAllianceWallRightSide;

  
 /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  private double previousPipelineTimestamp = 0;

  
  private static final Notifier photonNotifier = new Notifier(VisionSubsystem.photonRunnable());
  
  VisionSubsystem vision;
  DifferentialDrivePoseEstimator poseEstimator;
  GyroSubsystem gyro;
  DrivetrainSubsystem drivetrainSubsystem;
  Field2d field2d;
   
  /** Creates a new PoseEstimator. */
  public PoseEstimator(GyroSubsystem gyro,DrivetrainSubsystem drivetrainSubsystem,VisionSubsystem vision,DifferentialDriveKinematics kinematics,DifferentialDrivePoseEstimator poseEstimator,Supplier<PoseEstimator> PositionSupplier,Supplier<Rotation2d> rotationSupplier ) {
    


    new DifferentialDrivePoseEstimator(kinematics,gyro.getRotation2d(), drivetrainSubsystem.getPositionLeftLead(), drivetrainSubsystem.getPositionRightLead(), new Pose2d(), stateStdDevs, visionMeasurementStdDevs);
  
    photonNotifier.setName("PhotonRunnable");
    photonNotifier.startPeriodic(0.02);
  }
  private Pose2d flipAlliance() {
    return flipAlliance().relativeTo(VisionConstants.FLIPPING_POSE);
  }
  public void setAlliance(Alliance alliance) {
    boolean allianceChanged = false;
    switch(alliance) {
      case Blue:
        allianceChanged = (originPosition == kRedAllianceWallRightSide);
        originPosition = kBlueAllianceWallRightSide;
        break;
      case Red:
        allianceChanged = (originPosition == kBlueAllianceWallRightSide);
        originPosition = kRedAllianceWallRightSide;
        break;
      default:
      // No valid alliance data. Nothing we can do about it      sawTag
      if (allianceChanged && vision.seeTarget()) {
    }
      // The alliance changed, which changes the coordinate system.
      // Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
      // needs to be transformed to the new coordinate system.
      var newPose = flipAlliance();
      poseEstimator.resetPosition(gyro.m_gyro.getRotation2d(),drivetrainSubsystem.getPositionLeftLead() ,drivetrainSubsystem.getPositionRightLead(), newPose);
      //nota para andres del futuro capaz y esta mal porque se necesitan en metros y no se si sean en metros
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    var PhotonPipelineResult = VisionSubsystem.camera.getLatestResult();
    var resultsTimestamp = PhotonPipelineResult.getTimestampSeconds();
    if (resultsTimestamp != previousPipelineTimestamp && PhotonPipelineResult.hasTargets()){
      previousPipelineTimestamp = resultsTimestamp;
      var target = PhotonPipelineResult.getBestTarget();
      var fiducialid = target.getFiducialId();
      if(target.getPoseAmbiguity() <= 0.2 && fiducialid >= 0 && fiducialid < 9) {
        var targetPose = VisionConstants.tagPoses[fiducialid];
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(VisionConstants.cameraOnRobot);
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultsTimestamp);
      }
    }
    //update pose estimator with drivetrain sensors
    poseEstimator.update(
      gyro.getRotation2d(), previousPipelineTimestamp, previousPipelineTimestamp);

       field2d.setRobotPose(getcurrentPose());
  }
  
  public Pose2d getcurrentPose(){
    return poseEstimator.getEstimatedPosition();
  }
}
