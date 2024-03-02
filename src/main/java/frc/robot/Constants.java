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
        public static final int intakeMotorPort = 9;
        public static final int climbMotorPort = 10;
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
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(Units.inchesToMeters(-9.0), Units.inchesToMeters(7.0), Units.inchesToMeters(28.0)), 
        new Rotation3d(0.0, 0.0, 180.0));
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
    public static class MotorPorts {
        public static final int kLeftShooterPort = 12;
        public static final int kRightShooterPort = 13;
        public static final int kPanningPort = 11;
        public static final int kLeftFlap = 14;
        public static final int kRightFlap = 15;
      }
    
    
      public static class ShooterConstants {
        public static final double kLeftShooterP = 0.0002;
        public static final double kLeftShooterI = 0;
        public static final double kLeftShooterIZone = 0;
        public static final double kLeftShooterD = 0.0;
        public static final double kLeftShooterFF = 0.000175;
    
        public static final double kRightShooterP = 0.0002;
        public static final double kRightShooterI = 0;
        public static final double kRightShooterIZone = 0;
        public static final double kRighthooterD = 0.0;
        public static final double kRightShooterFF = 0.000170;
    
        public static final double kMinPIDOutput = -1.0;
        public static final double kMaxPIDOutput = 1.0;
    
        public static final double kVelocityTolerance = 30;   
    
        public static final double kAmpSpeed = 100;
        public static final double kSpeakerCloseSpeed = 1000;
        public static final double kSpeakerMidSpeed = 3000;
        public static final double kSpeakerFarSpeed = 6700;
        public static final double kIntakeSpeed = -1500;
      }
    
      public static class FlapValues {
        public static final double home = 0.0;
        public static final double ampValue= 4.0 ; //rot. to encoder values
        public static final double speakerValue = 8.0 ;
    
        public static final double kLeftFlapP = 0.1;
        public static final double kLeftFlapI = 0;
        public static final double kLeftFlapIZone = 0;
        public static final double kLeftFlapD = 0;
        public static final double kLeftFlapFF = 0.000170;
    
        public static final double kRightFlapP = .06;
        public static final double kRightFlapI = 0;
        public static final double kRightFlapIZone = 0;
        public static final double kRightFlapD = 0;
        public static final double kRightFlapFF = 0.000170;
    
        public static final double kRight90 = 18.523756;
        public static final double kLeft90 = 17.880909;
        public static final double kRight45 = kRight90 / 2;
        public static final double kLeft45 = kLeft90 / 2;
        public static final double kSlantLLeft = kLeft45;
        public static final double kSlantLRight = 26.690289;
        public static final double kSlantRLeft = 25.356977;
        public static final double kSlantRRight = kLeft45;
      }
    
      public static class DigitalInputs {
        public static final int kLeftLimitSwitch = 0;
        public static final int kRightLimitSwitch = 1;
        public static final int kLClimbLimitSwitch = 2;
        public static final int kRClimbLimitSwitch = 3;
      }

      public static class AimValues {
        public static double kAimP = 0.0;
        public static double kAimI = 0.0;
        public static double kAimIZone = 0.0;
        public static double kAimD = 0.0;
        public static double kAimFF = 0.0;
        public static double kAmpAim = 0.0;
      }
    }
