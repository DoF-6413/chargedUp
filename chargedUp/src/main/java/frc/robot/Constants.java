// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAuxControllerPort = 1;
  }

  public static class DrivetrainConstants {
      // Todo: Remove rightFollower2 and leftFollower2 when moving to four motors
      public enum DriveMotor{
        leftLead(4),
        leftFollower1(5),
        rightLead(2),
        rightFollower1(3);

      public final int CAN_ID;

      DriveMotor(int value) {
        CAN_ID = value;
      }
    }

    public static final boolean kRightInverted = true;
    public static final boolean kLeftInverted = false;

    // PID Controlls for Forawrds and Backwards (Values from SysID)
    public static final double kMoveP = 0.0057008;
    public static final double kMoveI = 0;
    public static final double kMoveD = 0;
    public static final double kMoveTolerance = 0.1;

    // Kinematics
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(21.5);

    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(25);

    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);

    // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(kMoveI, kMoveD, WHEEL_BASE);
    // // Convert to wheel speeds
    // DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    // // Left velocity
    // double leftVelocity = wheelSpeeds.leftMetersPerSecond;
    // // Right velocity
    // double rightVelocity = wheelSpeeds.rightMetersPerSecond;

    // Facts about the Drivetrain
    // Number of motors within 1 gearbox (controlling the drivetrain)
    public static final int knumMotors = 3;
    public static final double kgearing = 8.68;
    // Moment of Inertia (aka force felt by robot when following path?)
    public static final double kMOI = 4;
    // Final Mass of Robot including Bumbers and Batteries
    public static final double kMass = 135;

    public static final double kwheelRadiusMeters = 3 * 0.0254;
    public static final int kWheelDiameter = 6;

    public static final int neoEncoderTicks = 42;
    public static final double kTicksToMetersConversionFactor = 39.3701;
    public static final double kTicksToFeetConversionFactor = 12;
    public static final double kTicksToMeters = 
    // 1.0 / neoEncoderTicks * 
    kWheelDiameter * kgearing * Math.PI/ kTicksToMetersConversionFactor;
    public static final double kTicksToFeet = 
    // 1.0 / neoEncoderTicks * 
    kWheelDiameter * kgearing * Math.PI/ kTicksToFeetConversionFactor;

    public static double loopPeriodSecs = 0.020;

    public static final double kStopMotors = 0;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    //todo: Updated Values using SysID
    public static final double ksVolts = 0.15643;
    public static final double kvVoltSecondPerMeter = 2.2445;
    public static final double kaVoltsSecondsSquaredPerMeter = 0.602247;
  }

  public static class VisionConstants {
    public static final double[] ksetpoints = new double[] { 0, 1.5, 3, 4 };
  }

  public static class AutoConstants {
    public static final double MAX_VELOCITY_PERCENT_OUTPUT = 0;
    public static final double MAX_ACCELERATION_PERCENT_OUTPUT = 0;

    public static final double kchargingStationDistance = 5;

  }

  public static class ArmConstants { 
    public enum ArmMotor {
      leftRotationMotor(6),
      rightRotationMotor(7);

      public final int CAN_ID;

      ArmMotor(int value) {
        CAN_ID = value;
      }
    }

    //Rotation System Facts
    public static final double kRotationGearing = 303.03; //106.06
    public static final int kRotationCurrentLimit = 15;
    //ticks to degrees
    public static final double kRotationPositionConversion = 360.0 /kRotationGearing;

    // Rotation Arm PID Values (Tune PID Before Feedforward)
    public static final double kRotationP = 0.25;
    public static final double kRotationI = 0;
    public static final double kRotationD = 0;
    public static final double kRotationTolerance = 0.5;

    //Trapazoidal Motion Profiling for Rotation Arm
    public static final double kArmMaxVelocity = 100;
    public static final double kArmMaxAcceleration = 100;

    public static final int kpotetiometerPort = 3;
    public static final double kpotetiometerRange = 180;
    public static final double kpotentiometerOffset = -64;
    //Arm Values
    public static final double kRotationZeroValue = 0;
    public static final double kHighPeak = 105;
    public static final double kMidBottom= 75;

    /**Human Player, Mid Cone Peak, High Cone Bottom Value */
    public static final double kHPMPHB = 87;
    public static final double kfloorCube = 35;
  }

  public static class TelescoperConstants{
    public static final int kTelescoperCANID = 8;

    // Telescoper PID Values
    public static final double kTelescoperP = 0.04;
    public static final double kTelescoperI = 0;
    public static final double kTelescoperD = 0;
    public static final double kTelescoperTolerance = 2;

    //Telescoper Current Limit
    public static final boolean kIsTelescoperCurrentLimitEnabled = true;
    public static final double kTelescoperContinuousCurrent = 25;
    public static final double kTelescoperPeakCurrent = 60;
    public static final double kTelescoperMaxTimeAtPeak = 5.0;

    public static final boolean kIsTelescoperInverted = false;
    public static final double kFalconTicks = 2048;
    public static final double kTelescopePositionConversionFactor = 1/kFalconTicks;
    public static final double kMaxTelescoperSpeed = 0.75;

    //Arm Values
    public static final double kMaxExtention = 50;
    /** Telescoper for Mid Cone, Ground Block, and  */
    public static final double kMCGB = 16;
    public static final double kGroundCone = 30;
  }

  public static class EndEffectorConstants{
    public static final int kEndEffectorCANID = 9;
  }

  public static class WristConstants{
    public static final double kgearing = (116.667); 
    public static final double kwristRotationPositionConversion =360.0 / kgearing;
    //Wrist PID Values 
    public static final double kWristP = 0.05;
    public static final double kWristI = 0;
    public static final double kWristD = 0;
    public static final double kWristTolerance = 10;

    
  }
}
