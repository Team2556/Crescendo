// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ShooterConstants.FlapState;
import frc.robot.Constants.ShooterConstants.ShooterState;
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
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.setShooterState(ShooterState.STOP);
        m_shooterSubsystem.setFlapState(FlapState.RESET);
    }

    @Override
    public void execute() {
        //ToDo Add calculations for auto-alignment with flaps
        switch (m_shooterSubsystem.getFlapState()) {
            case RESET -> m_shooterSubsystem.flapHome();
            case STRAIGHT -> m_shooterSubsystem.setFlapPosition(kLeft90, kRight90);
            case AUTO -> {
                Pose2d pose = poseSubsystem.getPose();
                double flapLeftAngle, flapRightAngle;
                double speakerY = 5.5;

                double deltaY = pose.getY() - speakerY;
                double hyp = Math.sqrt(deltaY * deltaY + pose.getX() * pose.getX());
                double sin = Math.sin(deltaY / hyp);
                double flapCenter = sin - pose.getRotation().getRadians();
                SmartDashboard.putNumber("Flap Center", Math.toDegrees(flapCenter));


            }
        }

        if(m_rightTrigger.getAsBoolean())
            m_shooterSubsystem.setShooterVelocity(m_shooterSubsystem.getShooterState().getVelocity());
        else
            m_shooterSubsystem.stop();

        SmartDashboard.putNumber("Shooter Angle", interpolation.calculate(poseSubsystem.getX(), poseSubsystem.getY()));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_shooterSubsystem.setShooterState(ShooterState.STOP);
        m_shooterSubsystem.setFlapState(FlapState.NONE);
    }
}