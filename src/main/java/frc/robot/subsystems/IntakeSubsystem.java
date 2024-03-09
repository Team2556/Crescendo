package frc.robot.subsystems;

import static frc.robot.Constants.Ports.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

/*
-"inspiration" from 2023 intake 
-pnumatics evaporated 
-Max to Flex
-limit switches added
 */

public class IntakeSubsystem extends SubsystemBase {
    private final static IntakeSubsystem instance = getInstance();
    private static CANSparkMax intake;
   // private final CANSparkFlex intake2 = new CANSparkFlex(intakeMotorPort2, CANSparkLowLevel.MotorType.kBrushless);
    
    /*LS stands for limit switch*/
    // DigitalInput LS_1 = new DigitalInput(0);
    // DigitalInput LS_2 = new DigitalInput(1);
    // DigitalInput LS_3 = new DigitalInput(2);

    int postion; 
    Timer timer = new Timer();

    public IntakeSubsystem() {
        intake = new CANSparkMax(intakeMotorPort, CANSparkLowLevel.MotorType.kBrushless);
        //intake1.restoreFactoryDefaults();
        //intake2.restoreFactoryDefaults();
        intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
        //intake2.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    }

    public static void setIntakeMotor(double speed) {
        /* SmartDashboard.putNumber("Intake Speed", speed); */
        // if (getPostion() == 3) {
        //     intakeOff();
        // }
       // else {
            intake.set(speed);
           // intake2.set(speed);
    //     }
    }
    
    public void intakeOff() {
        /* SmartDashboard.putNumber("Intake Speed", 0.0); */
        intake.set(0.0);
       // intake2.set(0.0);
    }

    public void moveToShooter() {
        timer.reset();
        timer.restart();
        if (timer.get() < .25) {
            intake.set(0.8);
        } else {
            intake.set(0.0);
        }
    }

    public Command intakeOnCommand() {
        return this.runOnce(() -> intake.set(0.8));
    }

    public Command intakeOffCommand() {
        return this.runOnce(() -> intake.set(0.0));
    }

    // public int getPostion() {
    //     /* Get Value of each limit switch, returns 0 if none are on */
    //     if (LS_3.get()){return 3;}
    //     else if (LS_2.get()){return 2;}
    //     else if (LS_1.get()){return 1;}
    //     else {return 0;}
    // }

    public static IntakeSubsystem getInstance() {
        if(instance == null)
            return new IntakeSubsystem();
        return instance;
    }
}
