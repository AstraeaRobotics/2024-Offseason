// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  IntakeSubsystem m_IntakeSubsystem;
  double preIntakeSpeed;
  double intakeSpeed;
  // double rollerSpeed;

  public IntakeNote(IntakeSubsystem m_IntakeSubsystem, double preIntakeSpeed, double intakeSpeed) {

    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.preIntakeSpeed = preIntakeSpeed;
    this.intakeSpeed = intakeSpeed;

    addRequirements(m_IntakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_IntakeSubsystem.setIntake(intakeSpeed);
    m_IntakeSubsystem.setPreIntake(preIntakeSpeed);
    SmartDashboard.putNumber("Inkake Velocity", m_IntakeSubsystem.getIntakeVelocity());
    SmartDashboard.putNumber("Pre-Intake Velocity", m_IntakeSubsystem.getPreIntakeVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.spinIntake(0);
    m_IntakeSubsystem.spinPreIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
