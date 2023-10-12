// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Stream;

import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

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
   static PhotonCamera camera = new PhotonCamera("FrontCamera");
  private static PhotonPipelineResult results = new PhotonPipelineResult();
  public static PhotonTrackedTarget target = results.hasTargets() ? results.getBestTarget() : null;
  public static Double yaw;
  public static Double pitch;
  public static Transform3d camToTarget;
  public static PoseEstimator photonrobotPoseEstimator;
  
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {

    PortForwarder.add(5800, "photonvision.local", 5800);

  }

  public PhotonPipelineResult photonResult(){
    return camera.getLatestResult();
  }
  
// public String getCorner(){
//   if (results.hasTargets() != false) {

//     return results.getBestTarget().getDetectedCorners().get(0).toString();  
//   }
//    else {
//     return "has no target";
//   }
// }

  // todo: provide portforwaring to connect without radio
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    results = camera.getLatestResult();
    updateSmartDashboard();
    seeTarget();
  }

  public void updateSmartDashboard() {
    // SmartDashboard.putBoolean("target?", seeTarget());
    // SmartDashboard.putNumber("Best fiducial", getBestFiducial());
    // SmartDashboard.putNumber("distance X", distanceFinder().getX());
    // SmartDashboard.putNumber("distance Y", distanceFinder().getY());
    // SmartDashboard.putNumber("distance Z", distanceFinder().getZ());
    if(seeTarget()){
    SmartDashboard.putNumber("getXvalue", this.photonResult().getBestTarget().getMinAreaRectCorners().get(0).x);
    SmartDashboard.putNumber("getYvalue", this.photonResult().getBestTarget().getMinAreaRectCorners().get(0).y);
  }
    // SmartDashboard.putNumber("photonTime", this.getTimestampSeconds());
    // SmartDashboard.putString("getcornerstarget", results.getBestTarget().getDetectedCorners().get(1).toString());
  }

  public boolean seeTarget() {
    if (results.hasTargets() == true) {
      target = results.getBestTarget();
      yaw = target.getYaw();
      pitch = target.getPitch();
      camToTarget = target.getBestCameraToTarget();
    }
    SmartDashboard.putBoolean("See Target", results.hasTargets());
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
