// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class DrivetrainConstants{
    public static final int[] kDrivetrainCANIDs = new int[] {1,2,3, 10, 11, 12};

    public static final boolean kRightInverted = false;
    public static final boolean kLeftInverted = true;
    
    // todo: update equation
    public static final int kWheelDiameter = 6;
    public static final double kTicksToFeat = 1.0 / 42. * kWheelDiameter * 6.11 * Math.PI / 12.;;
    public static final double kStopMotors = 0;

    //PID Controlls for Forawrds and Backwards
    public static final double kMoveP = 25;
    public static final double kMoveI = 0;
    public static final double kMoveD = 0;
    public static final double kMoveTolerance = 1;

    //PID Controlls for Turning
    public static final double kTurnP = 0;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnTolerance = 0;

    // Kinematics
      // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(20.733);
        
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(20.733);

    public static final DifferentialDriveKinematics kinematics = 
    new DifferentialDriveKinematics(kTrackWidth);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(kMoveI, kMoveD, WHEEL_BASE);
// Convert to wheel speeds
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
// Left velocity
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;
// Right velocity
    double rightVelocity = wheelSpeeds.rightMetersPerSecond;

  }

  public static class AutoConstants{
    public static final double MAX_VELOCITY_PERCENT_OUTPUT = 0;
    public static final double MAX_ACCELERATION_PERCENT_OUTPUT = 0;

    public static final double kchargingStationDistance = 5;
    //Facts about the Drivetrain
    //Number of motors within 1 gearbox (controlling the drivetrain)
    public static final int knumMotors = 3;
    public static final double kgearing =  6.11;
    //Moment of Inertia (aka force felt by robot when following path?)
    public static final double kMOI = 4;
    //Final Mass of Robot including Bumbers and Batteries
    public static final double kMass = 135;
    public static final double kwheelRadiusMeters = 3 * 0.0254;
    public static final double ktrackWidth = 21.87;

    public static double loopPeriodSecs = 0.020;
  }
  public static class VisionConstants {
    public static final double[] ksetpoints = new double[] {0, 1.5, 3, 4};
    


  }
}
