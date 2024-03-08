// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import static frc.robot.Constants.Ports;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private final static ClimbSubsystem instance = getInstance();

  public final double climbSpeed = 0.5;

  private final CANSparkMax climbMotor = new CANSparkMax(Ports.climbMotoPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final RelativeEncoder climbEncoder = climbMotor.getEncoder();

  private final DigitalInput LeftLimit = new DigitalInput(Ports.LClimbLS);
  private final DigitalInput RightLimit = new DigitalInput(Ports.RClimbLS);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    disableArms();
  }

  public void extendArms() {
    if (climbEncoder.getPosition() < 15) {
      climbMotor.set(climbSpeed);
    } else {
      disableArms();
    }
  }

  public void retractArms() {
    if (!LeftLimit.get() && !RightLimit.get()) {
      climbMotor.set(-climbSpeed);
    } else {
      disableArms();
    }
  }

  public void disableArms() {
    climbMotor.set(0);
  }

  public static ClimbSubsystem getInstance() {
    if (instance == null) {
      return new ClimbSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
