package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ResetFlaps extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public ResetFlaps() {
        addRequirements(shooterSubsystem);
    }
    @Override
    public void initialize() {
        super.initialize();
        shooterSubsystem.resetFlaps();
    }

    @Override
    public void execute() {
        super.execute();
        shooterSubsystem.flapHome();
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.leftHomeFlag && shooterSubsystem.rightHomeFlag;
    }
}
