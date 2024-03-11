// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShooterInterpolation;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.ShooterConstants.rightFlapDegrees;

public class ShootCommand extends Command {
    /** Creates a new ShootCommand. */
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final PoseSubsystem poseSubsystem = PoseSubsystem.getInstance();
    private final ShooterInterpolation interpolation;
    private final BooleanSupplier m_rightTrigger, m_rightBumper;

    public ShootCommand(BooleanSupplier rightTrigger, BooleanSupplier rightBumper) {
        m_rightTrigger = rightTrigger;
        m_rightBumper = rightBumper;
        interpolation = new ShooterInterpolation();
        addRequirements(m_shooterSubsystem);
        SmartDashboard.putNumber("Shooter Pitch Angle Input", 0);
        SmartDashboard.putNumber("Flap Test Angle", 5);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.setShooterState(ShooterState.STOP);
        m_shooterSubsystem.resetFlaps();
        m_shooterSubsystem.setPitchState(PitchState.AUTO);
    }

    @Override
    public void execute() {
        switch (m_shooterSubsystem.getFlapState()) {
            case RESET -> m_shooterSubsystem.flapHome();
            case STRAIGHT -> m_shooterSubsystem.setFlapPosition(kLeft90, kRight90);
            case AUTO -> {
                Pose2d pose = poseSubsystem.getPose();
                double flapLeftAngle = kLeft90, flapRightAngle = kRight90;
                double speakerY = 5.5;

                double deltaY = pose.getY() - speakerY;
                double hyp = Math.sqrt(deltaY * deltaY + pose.getX() * pose.getX());
                double sin = Math.sin(deltaY / hyp);
                double flapCenter = Math.toDegrees(sin - pose.getRotation().getRadians());
                SmartDashboard.putNumber("Flap Center", flapCenter);
                // Verify robot's angle is not outside the max angle the flaps should align at.
                if(!(Math.abs(flapCenter) > kMaxFlapAngle)) {
                    double v = 78.0 * Math.sin(Math.toRadians(flapCenter));
                    flapRightAngle = (90 + v) * rightFlapDegrees;
                    flapLeftAngle = (90 - v) * leftFlapDegrees;
                }
                m_shooterSubsystem.setFlapPosition(flapLeftAngle, flapRightAngle);
            }
            case INTAKE -> {
                double flapRightAngle = 45 * rightFlapDegrees;
                double flapLeftAngle = 45 * leftFlapDegrees;
                m_shooterSubsystem.setFlapPosition(flapLeftAngle, flapRightAngle);
            }
            case TEST -> {
                double angle = SmartDashboard.getNumber("Flap Test Angle", 5);
                double flapRightAngle = (90 + angle) * rightFlapDegrees;
                double flapLeftAngle = (90 + angle) * leftFlapDegrees;
                m_shooterSubsystem.setFlapPosition(flapLeftAngle, flapRightAngle);
            }
        }

        switch (m_shooterSubsystem.getShooterState()) {
            case INTAKE -> m_shooterSubsystem.setSpeed(kShooterIntakeSpeed);
            case AMP -> m_shooterSubsystem.setSpeed(kAmpSpeed);
            case SPEAKER -> m_shooterSubsystem.setShooterVelocity(kSpeakerVelocity);
            case STOP -> m_shooterSubsystem.stop();
        }

        switch (m_shooterSubsystem.getPitchState()) {
            case AUTO -> {
                Pose2d pose = poseSubsystem.getPose();
                Pose3d speaker = new Pose3d(0.3, 5.5, Units.inchesToMeters(78.0), new Rotation3d());
                Transform3d shooterPivot = new Transform3d(Units.inchesToMeters(3.5), 0.0, Units.inchesToMeters(6.0), new Rotation3d());
                Pose3d deltaPose = new Pose3d(pose).plus(shooterPivot);
                double deltaX = deltaPose.getX() - speaker.getX(), deltaY = deltaPose.getY() - speaker.getY();
                double b = Math.sqrt(deltaX * deltaX + deltaY * deltaY), a = speaker.getZ() - shooterPivot.getZ();
                double angle = Math.toDegrees(Math.atan(a / b));
                m_shooterSubsystem.setPitchPosition(angle + kPitchMinimumAngle);
            }
            case SPEAKER -> m_shooterSubsystem.setPitchPosition(kPitchSpeakerPosition);
            case VERTICAL -> m_shooterSubsystem.setPitchPosition(kPitchVerticalPosition);
            case AMP -> m_shooterSubsystem.setPitchPosition(kPitchAmpPosition);
            case INTAKE -> m_shooterSubsystem.setPitchPosition(kPitchIntakePosition);
            case NONE -> m_shooterSubsystem.stopPitch();
            case TEST -> m_shooterSubsystem.setPitchPosition(SmartDashboard.getNumber("Shooter Pitch Angle Input", 0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_shooterSubsystem.setShooterState(ShooterState.STOP);
        m_shooterSubsystem.setFlapState(FlapState.NONE);
    }
}