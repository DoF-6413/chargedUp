// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int[] kDrivetrainCANIDs = new int[] {1,2,3,4};

    public static final boolean kRightInverted = true;
    public static final boolean kLeftInverted = false;

    //PID Values

    //Potential
    public static final double kP = 0;
    //Intergral
    public static final double kI = 0;
    //Derivative
    public static final double kD = 0;
    public static final double kTolerance = 0;

    //FeedForward Values (Get these in robot characterization tool)

    //Volts
    public static final double kS = 0;
    //Volts Multiplied bu Seconds Over Distance
    public static final double kV = 0;
    //Volts Multiplies by Seconds Squared Over Distance
    public static final double kA = 0;

    // todo: update equation
    public static final double kTicksToFeat = 0;
  }
  
  public static class FiducialConstants{
    public static final double[] kArrowValues = new double[] {0, 21};
  }
}
