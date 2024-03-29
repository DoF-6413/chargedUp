// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class colorSensor extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensorV3 = new ColorSensorV3 (i2cPort);

  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kpurple = new Color(0.2502, 0.2502, 0.5002);
  private final Color kyellow = new Color(0.5315, 0.4438, 0.02515);
  /** Creates a new colorSensor. */
  public colorSensor() {
    m_colorMatcher.addColorMatch(kpurple);
    m_colorMatcher.addColorMatch(kyellow);
  }

  @Override
  public void periodic() {
    Color detectedColor = m_colorSensorV3.getColor();

    String colorString; //DO NOT DELETE, this is used in lines 40 and 43
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kpurple){
      colorString = "cube";
      // System.out.println("cube");
    } else if (match.color == kyellow){
      colorString = "cone";
      // System.out.println("cone");
    }


    double IR = m_colorSensorV3.getIR();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    

    int proximity = m_colorSensorV3.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);

  }
}
