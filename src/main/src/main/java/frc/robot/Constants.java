// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
  }

  public static class SwerveModuleConstants {
    public static final double lFX = 11.5;
    public static final double lFY = 12;
    public static final double lRX = 11.5;
    public static final double lRY = -12;
    public static final double rFX = -11.5;
    public static final double rFY = 12;
    public static final double rRX = -11.5;
    public static final double rRY = -12;

    public static final double kTurningEncoderPositionFactor = 1.0;
    public static final double kTurningEncoderVelocityFactor = 1.0;

    public static final double kTurningP = 0.0;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;
    public static final double kTurningFF = 0.0;

    public static final double kDrivingP = 0.0;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;
    // public static final double kDrivingFF = 0.0;

    // public static final int kDriveLF = 1;
    // public static final int kDriveLR = 3;
    // public static final int kDriveRF = 7;
    // public static final int kDriveRR = 5;

    // public static final int kTurnLF = 2;
    // public static final int kTurnLR = 4;
    // public static final int kTurnRF = 8;
    // public static final int kTurnRR = 6;

    // public static final int kCANcoderLF = 2;
    // public static final int kCANcoderLR = 4;
    // public static final int kCANcoderRF = 8;
    // public static final int kCANcoderRR = 6;
  }
}
