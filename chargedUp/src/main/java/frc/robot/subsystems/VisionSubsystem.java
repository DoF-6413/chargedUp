// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  PhotonCamera camera = new PhotonCamera("photonvision");
  private PhotonPipelineResult results; 
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    results = camera.getLatestResult();
  }

  public boolean seeTarget(){
    return results.hasTargets();
  }
}
