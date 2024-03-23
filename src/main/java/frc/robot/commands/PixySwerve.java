// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PixycamSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;

public class PixySwerve extends Command {
  private boolean check = false;
  private String[] objLocation;
  private float[] XandY = { 0, 0 };
  private final SwerveSubsystem swerve;
  private final PixycamSubsystem m_vision;
  private Trigger btnPress; // controller button
  private Translation2d translation = new Translation2d(0, 0);

 /*  public void PixycamSubsystem(Trigger bs) {
    addRequirements(m_vision);
    
  }*/

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    check = false;
  }

  public PixySwerve(PixycamSubsystem pixySubsystem, SwerveSubsystem swerve, Trigger btnPressTrigger) {
    this.swerve = swerve;
    m_vision = pixySubsystem;
    this.btnPress = btnPressTrigger;
    addRequirements(swerve, pixySubsystem);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try { // Call pixy values, convert them into a new array
      objLocation = m_vision.getPixyValue();
      if (btnPress.getAsBoolean()) {
        SmartDashboard.putString("X", objLocation[0]);
        SmartDashboard.putString("Y", objLocation[1]);
        XandY[1] = Float.parseFloat(objLocation[1]);
        SmartDashboard.putNumber("Number array Y", XandY[1]);
        if ((-8 > m_vision.pixy_center) || (m_vision.pixy_center > 8)) {
          m_vision.readData();
          m_vision.calculateAngle();

          swerve.drive(translation, m_vision.calculateAngle(), true);
        } else {
          swerve.drive(translation, 0, true);
        }

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
