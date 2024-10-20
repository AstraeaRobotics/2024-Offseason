// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.LimelightUtil;

public class PivotAlign extends Command {
  /** Creates a new PivotAlign. */
  ShooterSubsystem m_ShooterSubsystem;
  double ta;
  double pivotSetpoint;
  public PivotAlign(ShooterSubsystem m_ShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ta = LimelightUtil.getTa();
    pivotSetpoint = LimelightUtil.getShooterAngle(ta);

    m_ShooterSubsystem.setDesiredPosition(pivotSetpoint);
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
