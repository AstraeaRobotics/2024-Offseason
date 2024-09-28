// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;

public class ManualRotation extends InstantCommand {
  private ShooterSubsystem m_shooterSubsystem;

  boolean increase;

  public ManualRotation(ShooterSubsystem shooterSubsystem, boolean increase) {
    this.m_shooterSubsystem = shooterSubsystem;
    this.increase = increase;
    addRequirements(m_shooterSubsystem);
  }

  @Override
  public void initialize() {
    if (increase) {
      m_shooterSubsystem.incrementPivot(0.5);
    }
    else m_shooterSubsystem.decrementPivot(0.5);

  }
}