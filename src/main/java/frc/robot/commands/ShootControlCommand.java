// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.FlapState;
import frc.robot.Constants.ShooterConstants.PitchState;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShooterInterpolation;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ShooterConstants.*;

public class ShootControlCommand extends Command {
    /** Creates a new ShootCommand. */
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final PoseSubsystem poseSubsystem = PoseSubsystem.getInstance();
    private final ShooterInterpolation interpolation;
    private final BooleanSupplier m_rightTrigger, m_rightBumper;

    public ShootControlCommand(BooleanSupplier rightTrigger, BooleanSupplier rightBumper) {
        m_rightTrigger = rightTrigger;
        m_rightBumper = rightBumper;
        interpolation = new ShooterInterpolation();
        addRequirements(m_shooterSubsystem);
        SmartDashboard.putNumber("Shooter Pitch Angle Input", 0);
        SmartDashboard.putNumber("Flap Test Angle", 5);
        SmartDashboard.putNumber("Flap Speed Scalar", 0.0);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.setShooterState(ShooterState.SPEAKER);
        m_shooterSubsystem.resetFlaps();
        m_shooterSubsystem.setPitchState(PitchState.DRIVE);

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        alliance.ifPresent(value -> ShooterSubsystem.red = value.equals(DriverStation.Alliance.Red));
    }

    @Override
    public void execute() {
        double leftVelocity = 0, rightVelocity = 50;
        switch (m_shooterSubsystem.getFlapState()) {
            case RESET -> m_shooterSubsystem.flapHome();
            case STRAIGHT -> m_shooterSubsystem.setFlapPosition(kLeft90, kRight90);
            case AUTO -> {
                Pair<Double, Double> flapAngles = m_shooterSubsystem.getFlapCalculatedAngle(poseSubsystem.getPose());
                m_shooterSubsystem.setFlapPosition(flapAngles.getFirst(), flapAngles.getSecond());
                double scalar = SmartDashboard.getNumber("Flap Speed Scalar", 0.0);
                leftVelocity = (((flapAngles.getFirst() - kLeft90) / kLeft90) * scalar) * kSpeakerVelocity;
                rightVelocity = (((flapAngles.getSecond() - kRight90) / kRight90) * scalar) * kSpeakerVelocity;
                leftVelocity = Math.max(leftVelocity, 0);
                rightVelocity = Math.max(rightVelocity, 0);
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
            case SIDE -> {
                m_shooterSubsystem.setFlapPosition(1.5, 1.5);
            }
        }

        switch (m_shooterSubsystem.getShooterState()) {
            case INTAKE -> {
                if(m_rightTrigger.getAsBoolean() || m_rightBumper.getAsBoolean())
                    m_shooterSubsystem.setSpeed(kShooterIntakeSpeed);
                else
                    m_shooterSubsystem.stop();
            }
            case AMP -> {
                if(m_rightTrigger.getAsBoolean() || m_rightBumper.getAsBoolean())
                    m_shooterSubsystem.setSpeed(kAmpSpeed);
                else
                    m_shooterSubsystem.stop();
            }
            case SPEAKER -> {
                rightVelocity = 50;
                if(m_rightTrigger.getAsBoolean() || m_rightBumper.getAsBoolean())
                    m_shooterSubsystem.setShooterVelocity(kSpeakerVelocity - leftVelocity, kSpeakerVelocity - rightVelocity);
                else
                    m_shooterSubsystem.stop();
            }
            case STOP -> m_shooterSubsystem.stop();
        }

        switch (m_shooterSubsystem.getPitchState()) {
            case AUTO -> {
                double angle = m_shooterSubsystem.getShooterCalculatedAngle(poseSubsystem.getPose());
                m_shooterSubsystem.setPitchPosition(angle + kPitchMinimumAngle);
            }
            case SPEAKER -> m_shooterSubsystem.setPitchPosition(kPitchSpeakerPosition);
            case VERTICAL -> m_shooterSubsystem.setPitchPosition(kPitchVerticalPosition);
            case AMP -> m_shooterSubsystem.setPitchPosition(kPitchAmpPosition);
            case INTAKE -> m_shooterSubsystem.setPitchPosition(kPitchIntakePosition);
            case DRIVE -> m_shooterSubsystem.setPitchPosition(kPitchDrivePosition);
            case NONE -> m_shooterSubsystem.stopPitch();
            case TEST -> m_shooterSubsystem.setPitchPosition(SmartDashboard.getNumber("Shooter Pitch Angle Input", 0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_shooterSubsystem.setShooterState(ShooterState.STOP);
        m_shooterSubsystem.setFlapState(FlapState.NONE);
        m_shooterSubsystem.setPitchState(PitchState.NONE);
    }
}