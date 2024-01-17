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
    private final CANSparkFlex intake = new CANSparkFlex(intakeMotorPort, CANSparkLowLevel.MotorType.kBrushless);

    /*(0,1,2...ect refer to ports) (LS stands for limit switch; F:1st S:2st L:3st)*/
    DigitalInput LSF = new DigitalInput(0);
    DigitalInput LSS = new DigitalInput(1);
    DigitalInput LSL = new DigitalInput(2);

    int postion; 

    public IntakeSubsystem() {
        intake.restoreFactoryDefaults();
        intake.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    }

    public void setIntakeMotor(double speed) {
        SmartDashboard.putNumber("Intake Speed", speed);
        intake.set(speed);
    }

    public int getPostion() {
        if (LSF.get()){postion = 1;}
        else if (LSS.get()){postion = 2;}
        else if (LSL.get()){postion = 3;}
        return postion;
    }

    public static IntakeSubsystem getInstance() {
        if(instance == null)
            return new IntakeSubsystem();
        return instance;
    }
}
