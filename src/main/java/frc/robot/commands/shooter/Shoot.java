// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {
  /** Creates a new ShooterIntake. */
  ShooterSubsystem m_ShooterSubsystem;
  double indexermotorSpeed;
  double topShooterSpeed;
  double bottomShooterSpeed;
  double m_shooterSpeed = 0;

  public Shoot(ShooterSubsystem m_ShooterSubsystem, double shooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_ShooterSubsystem);

    this.m_ShooterSubsystem = m_ShooterSubsystem;
    m_shooterSpeed = shooterSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_ShooterSubsystem.spinIndexer(.5);
    // m_ShooterSubsystem.prepShoot();
    // m_ShooterSubsystem.spinBottomShooter(80, 8.75);
    // m_ShooterSubsystem.spinTopShooter(80, 8.75);
    // // 80 RPS = 8.75 FF

    m_ShooterSubsystem.spinBottomShooter(m_shooterSpeed);
    m_ShooterSubsystem.spinTopShooter(m_shooterSpeed);

    // if (m_ShooterSubsystem.getShooterError() < 5) {
      m_ShooterSubsystem.spinIndexer(-.8);
    // }
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
