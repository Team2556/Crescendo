// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
  private final BooleanEvent rightTrigger;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, BooleanEvent rightTrigger) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);

    this.rightTrigger = rightTrigger;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.intakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.rightTrigger.getAsBoolean()) {
      intakeSubsystem.setIntakeMotor(0.8);
    } else {
      intakeSubsystem.setIntakeMotor(0);
    }
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
