package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAngleTest extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private double prevP,prevI,prevD,prevS,prevG,prevV,
            p = Constants.ShooterConstants.pitchShooterPIDF.p,
            i = Constants.ShooterConstants.pitchShooterPIDF.i,
            d = Constants.ShooterConstants.pitchShooterPIDF.d,
            kG = Constants.ShooterConstants.kG,
            kS = Constants.ShooterConstants.kS,
            kV = Constants.ShooterConstants.kV;

    public ShooterAngleTest() {
        addRequirements(shooterSubsystem);
        SmartDashboard.putNumber("Shooter Angle P", Constants.ShooterConstants.pitchShooterPIDF.p);
        SmartDashboard.putNumber("Shooter Angle I", Constants.ShooterConstants.pitchShooterPIDF.i);
        SmartDashboard.putNumber("Shooter Angle D", Constants.ShooterConstants.pitchShooterPIDF.d);
        SmartDashboard.putNumber("Shooter Angle kS", Constants.ShooterConstants.kG);
        SmartDashboard.putNumber("Shooter Angle kG", Constants.ShooterConstants.kS);
        SmartDashboard.putNumber("Shooter Angle kV", Constants.ShooterConstants.kV);
        SmartDashboard.putNumber("Shooter Angle Input", Constants.ShooterConstants.kPitchDrivePosition);
    }

    @Override
    public void initialize() {
        super.initialize();
        prevP = p;
        prevI = i;
        prevD = d;
        prevS = kS;
        prevG = kG;
        prevV = kV;
    }

    @Override
    public void execute() {
        super.execute();
        p = SmartDashboard.getNumber("Shooter Angle P", prevP);
        i = SmartDashboard.getNumber("Shooter Angle I", prevI);
        d = SmartDashboard.getNumber("Shooter Angle D", prevD);
        kS = SmartDashboard.getNumber("Shooter Angle kS", prevS);
        kG = SmartDashboard.getNumber("Shooter Angle kG", prevG);
        kV = SmartDashboard.getNumber("Shooter Angle kV", prevV);

        double pos = SmartDashboard.getNumber("Shooter Angle Input", Constants.ShooterConstants.kPitchDrivePosition);

        if(p != prevP || i != prevI || d != prevD) {
            shooterSubsystem.setPitchPID(p, i, d);
            prevP = p;
            prevI = i;
            prevD = d;
        }

        if(kS != prevS || kG != prevG || kV != prevV) {
            shooterSubsystem.setPitchFeedforward(kS, kG, kV);
            prevS = kS;
            prevG = kG;
            prevV = kV;
        }

        shooterSubsystem.setPitchPosition(pos);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooterSubsystem.stopPitch();
    }
}