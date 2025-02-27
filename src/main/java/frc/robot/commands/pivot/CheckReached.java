// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterStates;
import frc.robot.subsystems.ShooterSubsystem;

public class CheckReached extends Command {
  ShooterSubsystem m_shooterSubsystem; 

  /** Creates a new CheckReached. */
  public CheckReached(ShooterSubsystem shooterSubsystem) {
    this.m_shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.m_shooterSubsystem.getPivotError() < 5;
  }
}
