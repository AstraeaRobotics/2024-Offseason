// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterIntake extends Command {
  /** Creates a new ShooterIntake. */
  ShooterSubsystem m_ShooterSubsystem;
  double indexermotorSpeed;
  double topShooterSpeed;
  double bottomShooterSpeed;

  public ShooterIntake(ShooterSubsystem m_ShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_ShooterSubsystem);

    this.m_ShooterSubsystem = m_ShooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.prepIntake();
    m_ShooterSubsystem.spinIndexer(0.8);
    m_ShooterSubsystem.spinBottomShooter(75);
    m_ShooterSubsystem.spinTopShooter(75);
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
