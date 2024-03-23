// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    public static final Pose2d blueSubwoofer = new Pose2d(1.35, 5.5, Rotation2d.fromDegrees(0.0));
    public static final Pose2d redSubwoofer = new Pose2d(15.1, 5.5, Rotation2d.fromDegrees(-180.0));
    public static final Pose2d blueSpeakerScore = new Pose2d(3.3, 4.8, Rotation2d.fromDegrees(-15.0));
    public static final Pose2d redSpeakerScore = new Pose2d(13.3, 4.8, Rotation2d.fromDegrees(180.0 + 15.0));
    public static final Pose2d blueAmp = new Pose2d(1.8, 7.6, Rotation2d.fromDegrees(90.0));
    public static final Pose2d redAmp = new Pose2d(14.7, 4.8, Rotation2d.fromDegrees(90.0));
    public static final Pose2d blueIntake = new Pose2d(15.5, 1.0, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d redIntake = new Pose2d(1.15, 1.0, Rotation2d.fromDegrees(-120.0));

    public static class Ports {
        public static final int kIntakePort = 9;
        public static final int kClimbPort = 10;
        public static final int kShooterPitch = 11;
        public static final int kLeftShooterPort = 12;
        public static final int kRightShooterPort = 13;
        public static final int kLeftFlap = 14;
        public static final int kRightFlap = 15;


        // Digital Inputs
        public static final int kLeftFlapLimitSwitch = 1;
        public static final int kRightFlapLimitSwitch = 0;
        public static final int kLeftClimbLimitSwitch = 2;
        public static final int kRightClimbLimitSwitch = 3;
        public static final int kIntakeBreakBeam = 4;
        public static final int kForwardLimitSwitch = 6;
        public static final int kBackwardLimitSwitch = 7;
    }

    public static class ShooterConstants {
        public static final PIDFConfig leftShooterPIDF = new PIDFConfig(0.002, 0.0, 0.0, 0.000175, 0.0);
        public static final PIDFConfig rightShooterPIDF = new PIDFConfig(0.002, 0.0, 0.0, 0.000170, 0.0);
        public static final PIDFConfig pitchShooterPIDF = new PIDFConfig(0.6, 0.0, 0.0, 0.0, 0.0);
        public static final double kS = 0.0, kG = 0.0, kV = 0.0;
        public static final double kMaxPIDOutput = 1.0;
        public static final double kMinPIDOutput = -1.0;

        public static final double kVelocityTolerance = 100;

        public static final double kFlapTolerance = 1.0;
        public static final double kPitchTolerance = 0.5;
        // Raw absolute encoder value
        public static final double kPitchAmpPosition = 22;
        public static final double kPitchVerticalPosition = 0;
        public static final double kPitchIntakePosition = 22;
        public static final double kPitchDrivePosition = 310;

        // Raw absolute encoder value
        public static final double kPitchSpeakerPosition = 325;
        public static final double kPitchMinimumAngle = 270;
        public static final double kPitchForwardLimit = 290;
        public static final double kPitchBackwardLimit = 20;
        // Degrees
        public static final double kMaxFlapAngle = 35.0;

        public enum ShooterState {
            STOP,
            INTAKE,
            AMP,
            SPEAKER
        }

        public static final double kSpeakerVelocity = 5800;
        public static final double kShooterIntakeSpeed = -0.1;
        public static final double kAmpSpeed = 0.125;


        // Flap constants
        public static final PIDFConfig leftFlapPIDF = new PIDFConfig(0.07, 0.0, 0.0, 0.000170, 0.0);
        public static final PIDFConfig rightFlapPIDF = new PIDFConfig(0.07, 0.0, 0.0, 0.000170, 0.0);
        // Encoder positions
        public static final double kLeft90 = 15.5;
        public static final double kRight90 = 15.5;
        // Conversion factors
        public static final double rightFlapDegrees = kLeft90 / 90.0;
        public static final double leftFlapDegrees = kRight90 / 90.0;

        public enum FlapState {
            NONE,
            RESET,
            STRAIGHT,
            INTAKE,
            TEST,
            AUTO,
            SIDE
        }

        public enum PitchState {
            NONE,
            VERTICAL,
            INTAKE,
            SPEAKER,
            AMP,
            AUTO,
            DRIVE,
            TEST
        }
    }

    public static final double kIntakeMaxSpeed = 1.0;
    public static final double kOuttakeMaxSpeed = -1.0;
    public static final double kElevatorMaxSpeed = 1.0;
}