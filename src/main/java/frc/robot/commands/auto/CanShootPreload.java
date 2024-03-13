package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.ShooterConstants.kPitchSpeakerPosition;
import static frc.robot.Constants.ShooterConstants.kSpeakerVelocity;

public class CanShootPreload extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    
    public CanShootPreload() {
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        shooterSubsystem.setShooterState(Constants.ShooterConstants.ShooterState.SPEAKER);
        shooterSubsystem.setPitchState(Constants.ShooterConstants.PitchState.SPEAKER);
    }

    @Override
    public void execute() {
        super.execute();
        shooterSubsystem.setPitchPosition(kPitchSpeakerPosition);
        shooterSubsystem.setShooterVelocity(kSpeakerVelocity);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.isOnTarget();
    }
}
