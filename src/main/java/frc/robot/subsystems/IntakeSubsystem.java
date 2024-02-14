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
    private final CANSparkFlex intake1 = new CANSparkFlex(intakeMotorPort1, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex intake2 = new CANSparkFlex(intakeMotorPort2, CANSparkLowLevel.MotorType.kBrushless);
    
    /*LS stands for limit switch*/
    DigitalInput LS_1 = new DigitalInput(0);
    DigitalInput LS_2 = new DigitalInput(1);
    DigitalInput LS_3 = new DigitalInput(2);

    int postion; 

    public IntakeSubsystem() {
        intake1.restoreFactoryDefaults();
        intake2.restoreFactoryDefaults();
        intake1.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        intake2.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    }

    public void setIntakeMotor(double speed) {
        /* SmartDashboard.putNumber("Intake Speed", speed); */
        if (getPostion() == 3) {
            intakeOff();
        }
        else {
            intake1.set(speed);
            intake2.set(speed);
        }
    }
    
    public void intakeOff() {
        /* SmartDashboard.putNumber("Intake Speed", 0.0); */
        intake1.set(0.0);
        intake2.set(0.0);
    }

    public int getPostion() {
        /* Get Value of each limit switch, returns 0 if none are on */
        if (LS_3.get()){return 3;}
        else if (LS_2.get()){return 2;}
        else if (LS_1.get()){return 1;}
        else {return 0;}
    }

    public static IntakeSubsystem getInstance() {
        if(instance == null)
            return new IntakeSubsystem();
        return instance;
    }
}
