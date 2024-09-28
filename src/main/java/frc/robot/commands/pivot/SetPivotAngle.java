// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;

public class SetPivotAngle extends InstantCommand {
  private ShooterSubsystem m_shooterSubsystem;

  double pivotPosition;
  boolean increase;

  public SetPivotAngle(ShooterSubsystem shooterSubsystem, double pivotPosition) {
    this.m_shooterSubsystem = shooterSubsystem;
    this.pivotPosition = pivotPosition;
    addRequirements(m_shooterSubsystem);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.setDesiredPosition(pivotPosition);
  }
}