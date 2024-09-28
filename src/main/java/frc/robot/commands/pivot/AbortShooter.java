// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterStates;
import frc.robot.subsystems.ShooterSubsystem;

public class AbortShooter extends Command {
 
  ShooterSubsystem m_shooter;

  public AbortShooter(ShooterSubsystem m_shooter) {
    this.m_shooter = m_shooter;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setState(ShooterStates.kNull);
    m_shooter.stopShooter();
  }
}
