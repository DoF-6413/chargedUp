// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;


import org.photonvision.*;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;;



public class VisionSubsystem extends SubsystemBase {

   static DifferentialDrive poseEstimatorDifferentialDrive;
   static PhotonCamera camera = new PhotonCamera("Logi_Webcam_C920e");
  private static PhotonPipelineResult results = new PhotonPipelineResult();
  public PhotonTrackedTarget target = results != null ? results.getBestTarget() : null;
  public Double yaw;
  public Double pitch;
  public Transform3d camToTarget;
  public static PoseEstimator photonrobotPoseEstimator;
  private final PoseStrategy m_poseStrategy;
  
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {

    PortForwarder.add(5800, "photonvision.local", 5800);

    int aprilTag1 = 1;
    int aprilTag2 = 2;
    int aprilTag3 = 3;
    int aprilTag4 = 4;
    int aprilTag5 = 5;
    int aprilTag6 = 6;
    int aprilTag7 = 7;
    int aprilTag8 = 8;



    PoseEstimator PoseEstimator = null;
    try {
      var layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // PV estimates will always be blue, they'll get flipped by robot thread
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      if (camera != null) {
        photonrobotPoseEstimator = new PoseEstimator(
            layout, m_poseStrategy.MULTI_TAG_PNP, camera, VisionConstants.cameraOnRobot.inverse());
      }
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonrobotPoseEstimator = null;
    }
    this.poseEstimatorDifferentialDrive = poseEstimatorDifferentialDrive;

  }

  public static PhotonPipelineResult photonResult(){
    return camera.getLatestResult();
  }
  public static Runnable photonRunnable(){
   if (poseEstimatorDifferentialDrive != null && camera != null && !RobotState.isAutonomous()) {
    var photonResults = camera.getLatestResult();

    if (results.hasTargets() && (results.targets.size() > 1 || results.targets.get(0).getPoseAmbiguity() < VisionConstants.kaprilTagAmbiguityThreshold)) {
      pose
      
      update(photonResults).ifPresent(estimatedRobotPose ->{
      
        var estimatedPose = estimatedRobotPose.estimatedPose;
        if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= VisionConstants.kfieldLengthMeters
        && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= VisionConstnats.kfieldWidthMeters) {
          atomicEstimatedRobotPose.set(estimatedRobotPose);
        }
      });
      }
    
   }



  }

  // todo: provide portforwaring to connect without radio
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    results = camera.getLatestResult();
    updateSmartDashboard();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("target?", seeTarget());
    SmartDashboard.putNumber("Best fiducial ID", getBestFiducialID());
    SmartDashboard.putNumber("distance X", distanceFinder().getX());
    SmartDashboard.putNumber("distance Y", distanceFinder().getY());
    SmartDashboard.putNumber("distance Z", distanceFinder().getZ());
  }

  public boolean seeTarget() {
    if (results.hasTargets() == true) {
      target = results.getBestTarget();
      yaw = target.getYaw();
      pitch = target.getPitch();
      camToTarget = target.getBestCameraToTarget();
    }
    return results.hasTargets();
  }

  public  int getBestFiducialID() {
    return seeTarget() == true ? results.getBestTarget().getFiducialId() : 0;
  }

  public PhotonTrackedTarget getBestFiducial() {
    return seeTarget() == true ? results.getBestTarget() : new PhotonTrackedTarget();
  }

  public Translation3d distanceFinder() {
    Transform3d m_default = new Transform3d();
    return seeTarget() == true ? results.getBestTarget().getBestCameraToTarget().getTranslation() :new Translation3d();
  }

  public double getTargetYaw(){

    System.out.println(yaw);
    return( yaw != null) ? yaw : null;
  }

  public boolean isHuge() {
    boolean grande = (this.seeTarget() == true) ? results.getBestTarget().getArea() > .5 : false;
    boolean huge = (this.seeTarget() == true) ? grande : false;
    return huge;
  }

  public double getZ() {
    if (results.hasTargets() == true) {
    return target.getYaw();
    } else {
      return 1000;
    }
  }


  

}
