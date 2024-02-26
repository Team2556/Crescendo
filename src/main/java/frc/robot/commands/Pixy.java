// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PixycamSubsystem;

public class Pixy extends Command {
  private boolean check = false;
  private String[] objLocation;
  private float[] XandY= {0,0};
  private final PixycamSubsystem m_vision = PixycamSubsystem.getInstance();
  private final Trigger btnPress; //controller button

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
    try { //Call pixy values, convert them into a new array
      objLocation = m_vision.getPixyValue();
      if (btnPress.getAsBoolean()) {
        SmartDashboard.putString("X", objLocation[0]);
        SmartDashboard.putString("Y", objLocation[1]);
        XandY[1]= Float.parseFloat(objLocation[1]);
        SmartDashboard.putNumber("Number array Y", XandY[1]);

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

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return check;
  }
}
