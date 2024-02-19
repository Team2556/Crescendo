package frc.robot.subsystems;

import static frc.robot.Constants.Ports.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
-"inspiration" from 2023 intake 
-pnumatics evaporated 
-Max to Flex
-limit switches added
 */

public class IntakeSubsystem extends SubsystemBase {
    private final static IntakeSubsystem instance = getInstance();
    private final CANSparkFlex intakeMotor = new CANSparkFlex(intakeMotorPort1, CANSparkLowLevel.MotorType.kBrushless);
    
    DigitalInput intakeLimitSwitch = new DigitalInput(1);

    public IntakeSubsystem() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    }

    public void setIntakeMotor(double speed) {
        SmartDashboard.putNumber("Intake Speed", speed);
        if (intakeLimitSwitch.get()) {
            intakeOff();
        }
        else {
            intakeMotor.set(speed);
        }
    }
    
    public void intakeOff() {
        SmartDashboard.putNumber("Intake Speed", 0.0);
        intakeMotor.set(0.0);
    }

    public static IntakeSubsystem getInstance() {
        if(instance == null)
            return new IntakeSubsystem();
        return instance;
    }
}
