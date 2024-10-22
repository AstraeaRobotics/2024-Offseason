// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.LimelightUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PivotAlign extends InstantCommand {
  ShooterSubsystem m_ShooterSubsystem;

  public PivotAlign(ShooterSubsystem m_ShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double desiredPosition = LimelightUtil.getShooterAngle(LimelightUtil.getTa());

    m_ShooterSubsystem.setDesiredPosition(desiredPosition);
    m_ShooterSubsystem.setManualOffset(0);
  }
}
