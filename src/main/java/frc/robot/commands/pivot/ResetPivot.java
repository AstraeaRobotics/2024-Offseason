// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ResetPivot extends Command {
  private ShooterSubsystem m_shooterSubsystem;


  public ResetPivot(ShooterSubsystem shooterSubsystem) {
    this.m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.setPivotCoast();
  }

  @Override
  public void execute() {
    m_shooterSubsystem.setPivotCoast();
    m_shooterSubsystem.setReset(true);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setPivotBrake();
    m_shooterSubsystem.setReset(false);
    m_shooterSubsystem.setPivotPosition(0);
  }


}