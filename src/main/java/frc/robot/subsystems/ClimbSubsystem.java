// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private final static ClimbSubsystem instance = getInstance();

  private final Talon climbMotor = new Talon(0);
  public final double climbSpeed = 0.5;

  private final DigitalInput LeftLimit = new DigitalInput(0);
  private final DigitalInput RightLimit = new DigitalInput(1);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    disableArms();;
  }

  public void extendArms() {
    climbMotor.set(climbSpeed);
  }

  public void retractArms() {
    if (!LeftLimit.get() && !RightLimit.get()) {
      climbMotor.set(-climbSpeed);
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
