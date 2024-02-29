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
    public static final double LEFT_Y_DEADBAND = 0.15;
    public static final double LEFT_X_DEADBAND = 0.15;
    public static final double RIGHT_X_DEADBAND = 0.15;
    }

    public static class MotorPorts {
        public static final int kLeftShooterPort = 11;
        public static final int kRightShooterPort = 12;
        public static final int kLeftFlap = 13;
        public static final int kRightFlap = 14;
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
        public static final int kLeftLimitSwitch = 1;
        public static final int kRightLimitSwitch = 0;
    }
}
//
//Dpad
//    up for 90*
//    down for 45*
//    right for right angle
//    left for left angle
//
//Start to reset
//
//right trigger to shoot
//left trigger intake