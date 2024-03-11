package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();

    @Override
    public void execute() {
        super.execute();

    }
}