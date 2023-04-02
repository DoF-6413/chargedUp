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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;



public class PoseEstimator extends SubsystemBase {

  private OriginPosition originPosition = kRedAllianceWallRightSide;

  
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

  
  
  VisionSubsystem m_visionSubsystem;
  static DifferentialDrivePoseEstimator poseEstimator;
  GyroSubsystem m_gyroSubsystem;
  DrivetrainSubsystem m_drivetrainSubsystem;
  Field2d field2d = new Field2d();
  Pose3d m_visionMeasurement;
  static double resultsTimestamp;
  /** Creates a new PoseEstimator. */
  public PoseEstimator(GyroSubsystem gyro,DrivetrainSubsystem drive,VisionSubsystem vision,
  DifferentialDriveKinematics kinematics ) {
     
      m_drivetrainSubsystem = drive;
      m_gyroSubsystem = gyro;
      m_visionSubsystem = vision;
      setAlliance(DriverStation.getAlliance());
    poseEstimator = new DifferentialDrivePoseEstimator(kinematics,gyro.getRotation2d(), drive.getPositionLeftLead(), drive.getPositionRightLead(), new Pose2d(), stateStdDevs, visionMeasurementStdDevs);
  

  }
  private static Pose2d flipAlliance(Pose2d pose2d) {
    return flipAlliance(pose2d).relativeTo(VisionConstants.FLIPPING_POSE);
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
      if (allianceChanged && m_visionSubsystem.seeTarget()) {
        //me que de dormido porfas ve el video y comntinuea
    
      // The alliance changed, which changes the coordinate system.
      // Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
      // needs to be transformed to the new coordinate system.
      var newPose = flipAlliance(getcurrentPose());
      poseEstimator.resetPosition(m_gyroSubsystem.getRotation2d(), m_drivetrainSubsystem.getPositionLeftLead() , m_drivetrainSubsystem.getPositionRightLead(), newPose);
      }
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    var PhotonPipelineResult = VisionSubsystem.camera.getLatestResult();
    resultsTimestamp = PhotonPipelineResult.getTimestampSeconds();
    if (resultsTimestamp != previousPipelineTimestamp && PhotonPipelineResult.hasTargets()){
      previousPipelineTimestamp = resultsTimestamp;
      var target = PhotonPipelineResult.getBestTarget();
      var fiducialid = target.getFiducialId();
      if(target.getPoseAmbiguity() <= 0.2 && fiducialid >= 0 && fiducialid < 9) {
       
        AprilTagFieldLayout atfl;
        try {
          atfl = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField().getTags(), 20, 20);
          Pose3d targetPose = atfl.getTagPose(fiducialid).get();
          Transform3d camToTarget = target.getBestCameraToTarget();
          Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
          m_visionMeasurement = camPose.transformBy(VisionConstants.cameraOnRobot);
        
        } catch (IOException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
    }

   
    //update pose estimator with drivetrain sensors

    poseEstimator.update(
      m_gyroSubsystem.getRotation2d(), m_drivetrainSubsystem.getPositionLeftLead(), m_drivetrainSubsystem.getPositionRightLead());
  
      //  field2d.setRobotPose(getcurrentPose());

      //  var smartDashboardPose = poseEstimator.getEstimatedPosition();
      //  if (originPosition == kRedAllianceWallRightSide){
      //   smartDashboardPose = flipAlliance(smartDashboardPose);
      //  }


      //  field2d.setRobotPose(smartDashboardPose);
  }
  
  public Pose2d getcurrentPose(){
    poseEstimator.addVisionMeasurement(m_visionMeasurement.toPose2d(), resultsTimestamp);
    SmartDashboard.putString("final Pose", poseEstimator.getEstimatedPosition().toString());
    SmartDashboard.putString("visionmeasure", m_visionMeasurement.toPose2d().toString());
    return poseEstimator.getEstimatedPosition();
  }
}
  