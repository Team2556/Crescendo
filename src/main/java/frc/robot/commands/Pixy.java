// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

public class Pixy extends Command {
  private boolean check = false;
  // private static Vision m_vision;
  private String[] objLocation;
  private float[] XandY= {0,0};
  private final Vision m_vision = Vision.getInstance();
  private final Trigger btnPress;

  public Pixy(Trigger bs) {
    addRequirements(m_vision);
    btnPress = bs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    check = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      objLocation = m_vision.getPixyValue();
      if (btnPress.getAsBoolean()) {
        SmartDashboard.putString("X", objLocation[0]);
        SmartDashboard.putString("Y", objLocation[1]);
        XandY[1]= Float.parseFloat(objLocation[1]);
        SmartDashboard.putNumber("Number array Y", XandY[1]);

        // m_vision.readData(objLocation);
        // m_vision.getPixyValue();
        // m_vision.getError();
      } else {
        SmartDashboard.getString("error", "Button not found");
      }
    } catch (Exception e) {
      SmartDashboard.putString("Error2", "Method failed!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // public static Vision getInstance() {
  // return vision;
  // // if(instance == null)
  // // return new Vision();
  // // return instance;
  // }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return check;
  }
}
