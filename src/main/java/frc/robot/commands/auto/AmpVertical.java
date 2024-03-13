package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.ShooterConstants.*;

public class AmpVertical extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    
    public AmpVertical() {
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        shooterSubsystem.setFlapState(FlapState.SIDE);
        shooterSubsystem.setShooterState(Constants.ShooterConstants.ShooterState.AMP);
        shooterSubsystem.setPitchState(Constants.ShooterConstants.PitchState.VERTICAL);
    }

    @Override
    public void execute() {
        super.execute();
        shooterSubsystem.setPitchPosition(kPitchVerticalPosition);
        shooterSubsystem.setFlapPosition(kLeft90, kRight90);
        shooterSubsystem.setShooter(kAmpSpeed);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.shooterPitchArrived();
    }
}
