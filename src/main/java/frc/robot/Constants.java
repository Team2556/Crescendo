// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import swervelib.parser.PIDFConfig;

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
        public static final double LEFT_Y_DEADBAND = 0.15;
        public static final double LEFT_X_DEADBAND = 0.15;
        public static final double RIGHT_X_DEADBAND = 0.15;

        public static final double LEFT_TRIGGER_DEADBAND = 0.3;
        public static final double RIGHT_TRIGGER_DEADBAND = 0.3;
    }

    public static final double SLOW_MAX_SPEED = Units.feetToMeters(4.0);
    public static final double SWERVE_MAX_SPEED = Units.feetToMeters(16.0);

    public static class Ports {
        public static final int kIntakePort = 9;
        public static final int kClimbPort = 10;
        public static final int kShooterPitch = 11;
        public static final int kLeftShooterPort = 12;
        public static final int kRightShooterPort = 13;
        public static final int kLeftFlap = 14;
        public static final int kRightFlap = 15;


        // Digital Inputs
        public static final int kLeftFlapLimitSwitch = 0;
        public static final int kRightFlapLimitSwitch = 1;
        public static final int kLeftClimbLimitSwitch = 2;
        public static final int kRightClimbLimitSwitch = 3;
        public static final int kIntakeBreakBeam = 4;
    }

    public static class ShooterConstants {
        public static final PIDFConfig leftShooterPIDF = new PIDFConfig(0.0002, 0.0, 0.0, 0.000175, 0.0);
        public static final PIDFConfig rightShooterPIDF = new PIDFConfig(0.0002, 0.0, 0.0, 0.000170, 0.0);
        public static final double kMaxPIDOutput = 1.0;
        public static final double kMinPIDOutput = -1.0;

        public static final double kVelocityTolerance = 30;

        public static final double kFlapTolerance = 1.0;
        public static final double kPitchTolerance = 1.0;
        public static final double kPitchAmpPosition = 315.0;

        public enum ShooterState {
            STOP(0.0),
            INTAKE(-1500.0),
            AMP(100.0),
            SPEAKER(6700.0);
            private final double velocity;
            ShooterState(double velocity) {
                this.velocity = velocity;
            }

            public double getVelocity() {
                return velocity;
            }
        }

        // Flap constants
        public static final PIDFConfig leftFlapPIDF = new PIDFConfig(0.07, 0.0, 0.0, 0.000170, 0.0);
        public static final PIDFConfig rightFlapPIDF = new PIDFConfig(0.07, 0.0, 0.0, 0.000170, 0.0);
        // Encoder positions
        public static final double kLeft90 = 17.880909;
        public static final double kRight90 = 18.523756;
        // Conversion factors
        public static final double leftFlapDegrees = kLeft90 / 90.0;
        public static final double rightFlapDegrees = kRight90 / 90.0;

        public enum FlapState {
            NONE,
            RESET,
            STRAIGHT,
            AUTO
        }
    }

    public static final double kIntakeMaxSpeed = 0.8;
    public static final double kOuttakeMaxSpeed = -0.5;
    public static final double kElevatorMaxSpeed = 1.0;
}