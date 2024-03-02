// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndividualShoot extends Command {
  private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
  private final PhotonSubsystem m_photonSubsystem = PhotonSubsystem.getInstance();
  private final IntakeSubsystem m_intakeSubsystem = IntakeSubsystem.getInstance();
  /** Creates a new IndividualShoot. */
  public IndividualShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem, m_photonSubsystem, m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setFlapPositionByTags(PhotonSubsystem.targetRotation);
    m_shooterSubsystem.setAimPositionByTags(PhotonSubsystem.shootAngle);
    m_shooterSubsystem.setVelocityByTags(PhotonSubsystem.targetX, 
                                         m_shooterSubsystem.lFlapEncoder.getPosition(), 
                                         m_shooterSubsystem.rFlapEncoder.getPosition());
    m_intakeSubsystem.moveToShooter();
    m_shooterSubsystem.setSpeed(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
