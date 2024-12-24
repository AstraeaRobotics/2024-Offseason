// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightUtil;

public class RotationalAlign extends Command {
  /** Creates a new AlignToShoot. */
  SwerveSubsystem m_SwerveSubsystem;
  public RotationalAlign(SwerveSubsystem m_SwerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    
    addRequirements(m_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double driveRotationOutput = LimelightUtil.getTx() * .06;
    double driveRotationOutput = LimelightHelpers.getTX("limelight");

    // if(LimelightUtil.getTa() != 0) {
    //   driveYOutput = (LimelightUtil.getTa() - 8) * 0.035;
    // }
    m_SwerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, driveRotationOutput, Rotation2d.fromDegrees(m_SwerveSubsystem.getHeading())));
    m_SwerveSubsystem.drive(null);
    double shooterAngle = LimelightUtil.getShooterAngle(LimelightUtil.getTa());
    SmartDashboard.putNumber("shooter angle", shooterAngle);
    SmartDashboard.putNumber("april tag area", LimelightUtil.getTa());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
