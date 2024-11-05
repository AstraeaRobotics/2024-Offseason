// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
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
<<<<<<< Updated upstream:src/main/java/frc/robot/commands/vision/RotationalAlign.java
    double driveYOutput = 0;
    double driveXOutput = LimelightUtil.getTx() * .05;
=======
    double driveRotationOutput = LimelightUtil.getTx() * .06;
>>>>>>> Stashed changes:src/main/java/frc/robot/commands/vision/AlignToShoot.java

    // if(LimelightUtil.getTa() != 0) {
    //   driveYOutput = (LimelightUtil.getTa() - 8) * 0.035;
    // }
    m_SwerveSubsystem.drive(0, 0, driveRotationOutput);
    double shooterAngle = LimelightUtil.getShooterAngle(LimelightUtil.getTa());
    SmartDashboard.putNumber("shooter angle", shooterAngle);
    SmartDashboard.putNumber("april tag area", LimelightUtil.getTa());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
