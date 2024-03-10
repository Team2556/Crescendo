// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShooterInterpolation;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ShooterConstants.*;

public class ShootCommand extends Command {
    /** Creates a new ShootCommand. */
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final PoseSubsystem poseSubsystem = PoseSubsystem.getInstance();
    private final ShooterInterpolation interpolation;
    private final BooleanSupplier m_rightTrigger;

    public ShootCommand(BooleanSupplier rightTrigger) {
        m_rightTrigger = rightTrigger;
        interpolation = new ShooterInterpolation();
        addRequirements(m_shooterSubsystem);
        SmartDashboard.putNumber("Shooter Amp Speed", .1);
        SmartDashboard.putNumber("Shooter Pitch Angle Input", 20);
        SmartDashboard.putNumber("Shooter Amp Angle", 5);
        SmartDashboard.putNumber("Shooter Intake Speed", -0.3);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.setShooterState(ShooterState.STOP);
        m_shooterSubsystem.resetFlaps();
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
                    flapRightAngle = (90 + 78.0 * Math.sin(Math.toRadians(flapCenter))) * leftFlapDegrees;
                    flapLeftAngle = (90 - 78.0 * Math.sin(Math.toRadians(flapCenter))) * rightFlapDegrees;
                }
                m_shooterSubsystem.setFlapPosition(flapLeftAngle, flapRightAngle);
            }
            case AMP -> {
                double angle = SmartDashboard.getNumber("Shooter Amp Angle", 5);
                double flapRightAngle = (90 + angle) * leftFlapDegrees;
                double flapLeftAngle = (90 + angle) * rightFlapDegrees;
                m_shooterSubsystem.setFlapPosition(flapLeftAngle, flapRightAngle);
            }
        }

        if(m_rightTrigger.getAsBoolean()) {
            if(m_shooterSubsystem.getShooterState().equals(ShooterState.AMP)) {
                m_shooterSubsystem.setShooter(SmartDashboard.getNumber("Shooter Amp Speed", 0.15));
            } else if(m_shooterSubsystem.getShooterState().equals(ShooterState.INTAKE)) {
                m_shooterSubsystem.setShooter(SmartDashboard.getNumber("Shooter Intake Speed", -0.1));
            } else
                m_shooterSubsystem.setShooterVelocity(m_shooterSubsystem.getShooterState().getVelocity());
        } else
            m_shooterSubsystem.stop();
//        m_shooterSubsystem.setPitchPosition(SmartDashboard.getNumber("Shooter Pitch Angle Input", 340));
        SmartDashboard.putNumber("Shooter Angle", interpolation.calculate(poseSubsystem.getX(), poseSubsystem.getY()));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_shooterSubsystem.setShooterState(ShooterState.STOP);
        m_shooterSubsystem.setFlapState(FlapState.NONE);
    }
}