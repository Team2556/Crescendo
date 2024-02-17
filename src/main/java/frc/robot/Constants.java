// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // ToDo Get proper port numbers.
    // Swerve ports are in 'deploy/swerve/modules/.'
    public static final class Ports {
        public static final int intakeMotorPort = 20;
        public static final int shooterLeftMotorPort = 22;
        public static final int shooterRightMotorPort = 24;
        public static final int climbMotorPort = 26;
    }
    // ToDo See if separate PID constants are necessary.
    public static final PIDFConfig shooterLeftPIDF = new PIDFConfig(0.0002, 0.0, 0.0, 0.000175, 0.0);
    public static final PIDFConfig shooterRightPIDF = new PIDFConfig(0.0002, 0.0, 0.0, 0.000170, 0.0);
    public static final double shooterVelocityError = 30; // RPM

    // ToDo Get proper values.
    public static final class ElevatorConfiguration {
        public static final double kDt = 0.02; // loop time
        public static final double kMaxVelocity = 1.75;
        public static final double kMaxAcceleration = 0.75;
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.01;
        public static final double kS = 1.1;
        public static final double kG = 1.2;
        public static final double kV = 1.3;
        // ToDo Apply proper conversions to climb encoder & get the fully climbed position.
        public static final double climbGoal = 0;
        public static final double downGoal = 0;
        public static final double climbGoalTolerance = 0.1;
    }

    public enum ShooterState {
        STOP(0.0),
        AMP(100.0),
        SUBWOOFER(1000.0),
        CLOSE(1500.0),
        MIDDLE(3000.0),
        FAR(5000.0);

        private final double velocity;

        ShooterState(double velocity) {
            this.velocity = velocity;
        }

        public double getVelocity() {
            return velocity;
        }
    }

    public static final double ROBOT_MASS = (50.0) * 0.453592; // lbs * kg per pound
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(Units.inchesToMeters(-9.0), Units.inchesToMeters(7.0), Units.inchesToMeters(28.0)), 
        new Rotation3d(0.0, 0.0, 180.0));
    public static final Transform3d CAMERA_TO_ROBOT = ROBOT_TO_CAMERA.inverse();
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

    public static final class Auton {

        public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

        public static final double MAX_ACCELERATION = 2;
    }

    public static final class Drivebase {

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants {

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.15;
        public static final double LEFT_Y_DEADBAND = 0.15;
        public static final double RIGHT_X_DEADBAND = 0.15;
        public static final double TURN_CONSTANT = 0.75;
    }

    public class FieldK {
        public static final double kFieldLength = 16.54;
        public static final double kFieldWidth = 8.21;

        public static final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        public static final int kBlueCenterTrapID = 14;
        public static final int kBlueLeftTrapID = 15;
        public static final int kBlueRightTrapID = 16;

        public static final int kBlueCenterTrapId = 14;
        public static final int kBlueLeftTrapId = 15;
        public static final int kBlueRightTrapId = 16;

        public static final int kRedLeftTrapId = 11;
        public static final int kRedRightTrapId = 12;
        public static final int kRedCenterTrapId = 13;

    }

}