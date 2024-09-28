// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAmp extends Command {
  ShooterSubsystem m_ShooterSubsystem;
  /** Creates a new ShootAmp. */
  public ShootAmp(ShooterSubsystem m_ShooterSubsystem) {
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    addRequirements(this.m_ShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_ShooterSubsystem.prepAmp();
    m_ShooterSubsystem.spinTopShooter(50);
    if (m_ShooterSubsystem.getShooterError() < 5) {
      m_ShooterSubsystem.spinIndexer(-.65);
    }
    SmartDashboard.putNumber("Shooter Velocity", m_ShooterSubsystem.getShooterVelocity());
    SmartDashboard.putNumber("Shooter Error: ", m_ShooterSubsystem.getShooterError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.spinIndexer(0);
    m_ShooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
