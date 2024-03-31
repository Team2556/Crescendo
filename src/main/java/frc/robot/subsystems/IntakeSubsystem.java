package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Ports.kIntakeBreakBeam;
import static frc.robot.Constants.Ports.kIntakePort;

public class IntakeSubsystem extends SubsystemBase {
    // Subsystem instance
    private final static IntakeSubsystem instance = getInstance();
    // Intake motor
    private final CANSparkMax intakeMotor;
    // Break beam sensor
    private final DigitalInput intakeLimitSwitch;
    private boolean hasRumbled = false;
    

    /**
     * Constructor to configure and initialize the motor, and break beam sensor.
     */
    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(kIntakePort, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.clearFaults();
        intakeMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        intakeMotor.setInverted(false);

        intakeLimitSwitch = new DigitalInput(kIntakeBreakBeam);

        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.addNumber("Intake Current", intakeMotor::getOutputCurrent);
    }

    /**
     * Sets the intake motor speed and stops when the break beam is triggered.
     * @param speed Speed for the motor.
     */
    public void setIntakeMotor(double speed) {
        if (!intakeLimitSwitch.get()) {
            if(!hasRumbled) {
                RobotContainer.shouldRumble = true;
                hasRumbled = true;
            }
            stop();
        } else {
            hasRumbled = false;
            set(speed);
        }
    }

    /**
     * Function to set the intake motor and output speed to the smart dashboard.
     * Does not incorporate the break beam, and should only be used for automated actions.
     * @param speed Speed for the motor.
     */
    public void set(double speed) {
        SmartDashboard.putNumber("Intake Speed", speed);
        Logger.recordOutput("Intake Speed", speed);
        intakeMotor.set(speed);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        set(0.0);
    }

    /**
     * Get the status of the intake's break beam.
     * @return Whether the break beam is triggered.
     */
    public boolean getIntakeLimitSwitch() {
        return intakeLimitSwitch.get();
    }

    public double getIntakeOutputCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("Intake Beam Break", getIntakeLimitSwitch());
        SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
    }

    /**
     * Get an instance of the intake subsystem.
     * @return An instance of the intake subsystem.
     */
    public static IntakeSubsystem getInstance() {
        if(instance == null)
            return new IntakeSubsystem();
        return instance;
    }
}