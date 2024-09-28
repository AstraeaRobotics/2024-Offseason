// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterStates;
import frc.robot.subsystems.ShooterSubsystem;

public class PivotPID extends Command {
  private ShooterSubsystem m_shooterSubsystem;
  /** Creates a new PivotPID. */
  public PivotPID(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem);
    m_shooterSubsystem.setState(ShooterStates.kNull);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetPosition = (m_shooterSubsystem.getState().getPivotSetpoint() < 0) ? m_shooterSubsystem.getPivotPosition() : m_shooterSubsystem.getState().getPivotSetpoint();

    if (Math.abs(m_shooterSubsystem.getPivotError()) < 5) m_shooterSubsystem.stopPivot();

    else m_shooterSubsystem.spinPivotMotor(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopPivot();
  }
}
