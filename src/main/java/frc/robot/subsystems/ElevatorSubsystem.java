package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private static ElevatorSubsystem instance = getInstance();

    private final TalonFX climbMotor;
    private final DigitalInput leftClimbLimitSwitch, rightClimbLimitSwitch;

    public ElevatorSubsystem() {
        climbMotor = new TalonFX(Constants.Ports.kClimbPort);
        climbMotor.clearStickyFaults();
        climbMotor.setNeutralMode(NeutralModeValue.Brake);
        climbMotor.setInverted(true);

        leftClimbLimitSwitch = new DigitalInput(Constants.Ports.kLeftClimbLimitSwitch);
        rightClimbLimitSwitch = new DigitalInput(Constants.Ports.kRightClimbLimitSwitch);
    }

    public void setClimbSpeedOr(double speed) {
        // Verify set speed is attempting to go down, and whether it is touching one of the limit switches. If so, stop.
        if (speed < 0 && (leftClimbLimitSwitch.get() || rightClimbLimitSwitch.get()))
            stop();
        else
            set(speed);
    }

    public void setClimbSpeedAnd(double speed) {
        if (speed < 0 && (leftClimbLimitSwitch.get() && rightClimbLimitSwitch.get()))
            stop();
        else
            set(speed);
    }

    private void set(double speed) {
        SmartDashboard.putNumber("Climb Speed", speed);
        climbMotor.set(speed);
    }

    public void stop() {
        climbMotor.set(0.0);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("Climb Left Limit", leftClimbLimitSwitch.get());
        SmartDashboard.putBoolean("Climb Right Limit", rightClimbLimitSwitch.get());
    }

    public static ElevatorSubsystem getInstance() {
         if(instance == null)
             instance = new ElevatorSubsystem();
         return instance;
     }
}
