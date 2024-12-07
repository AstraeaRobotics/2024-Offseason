// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class TestPreIntake extends Command {
  /** Creates a new TestPreIntake. */
  IntakeSubsystem m_IntakeSubsystem;
  double voltage;
  public TestPreIntake(IntakeSubsystem m_IntakeSubsystem, double voltage) {
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.voltage = voltage;

    addRequirements(m_IntakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_IntakeSubsystem.setVoltagePreInt(voltage);

    m_IntakeSubsystem.preIntakeEncoder.setVelocityConversionFactor(.0167);
    SmartDashboard.putNumber("Pre-intake Velocity", m_IntakeSubsystem.getPreIntakeVelocity());
  }

  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setVoltagePreInt(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
