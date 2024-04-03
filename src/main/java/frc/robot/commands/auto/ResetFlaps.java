package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.ShooterConstants.kPitchSpeakerPosition;
import static frc.robot.Constants.ShooterConstants.kSpeakerVelocity;

public class ResetFlaps extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public ResetFlaps() {
        addRequirements(shooterSubsystem);
    }
    @Override
    public void initialize() {
        super.initialize();
        shooterSubsystem.resetFlaps();
        shooterSubsystem.setShooterState(Constants.ShooterConstants.ShooterState.SPEAKER);
        shooterSubsystem.setPitchState(Constants.ShooterConstants.PitchState.SPEAKER);
    }

    @Override
    public void execute() {
        super.execute();
        shooterSubsystem.flapHome();
        shooterSubsystem.setPitchPosition(kPitchSpeakerPosition);
        shooterSubsystem.setShooterVelocity(kSpeakerVelocity);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.leftHomeFlag && shooterSubsystem.rightHomeFlag;
    }
}
