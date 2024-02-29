// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ShooterConstants.FlapState;
import frc.robot.Constants.ShooterConstants.ShooterState;
import static frc.robot.Constants.ShooterConstants.*;

public class ShootCommand extends Command {
    /** Creates a new ShootCommand. */
    private final ShooterSubsystem m_shooterSubsystem;

    public ShootCommand(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
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
        }

        m_shooterSubsystem.setShooterVelocity(m_shooterSubsystem.getShooterState().getVelocity());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_shooterSubsystem.setShooterState(ShooterState.STOP);
        m_shooterSubsystem.setFlapState(FlapState.NONE);
    }
}