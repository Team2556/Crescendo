package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class ClimbSubsystem {
  private final static ClimbSubsystem instance = getInstance();

    Talon climbMotor = new Talon(0);
    double climbSpeed = 0.5;

    DigitalInput LeftLimit = new DigitalInput(0);
    DigitalInput RightLimit = new DigitalInput(1);

    // PIDController pid = new PIDController(1, 0.1, 0.01);

    public void extendArms() {
      climbMotor.set(climbSpeed);
    }
    
    public void disableArms () {
      climbMotor.set(0);
    }

    public void lowerArms() {
      if (LeftLimit.get() && RightLimit.get()) {
        climbMotor.set(0);
      }
      else {
        climbMotor.set(-climbSpeed);
      }
    }

    public static ClimbSubsystem getInstance() {
      if(instance == null)
          return new ClimbSubsystem();
      return instance;
  }
}