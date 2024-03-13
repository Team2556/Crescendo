package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.ShooterConstants.*;

public class PreparePreload extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public PreparePreload() {
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        shooterSubsystem.setFlapState(Constants.ShooterConstants.FlapState.STRAIGHT);
        shooterSubsystem.setShooterState(Constants.ShooterConstants.ShooterState.SPEAKER);
        shooterSubsystem.setPitchState(Constants.ShooterConstants.PitchState.SPEAKER);
    }

    @Override
    public void execute() {
        super.execute();
        shooterSubsystem.setFlapPosition(kLeft90, kRight90);
        shooterSubsystem.setPitchPosition(kPitchSpeakerPosition);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.flapsArrived(kLeft90, kRight90) && shooterSubsystem.shooterPitchArrived();
    }
}
