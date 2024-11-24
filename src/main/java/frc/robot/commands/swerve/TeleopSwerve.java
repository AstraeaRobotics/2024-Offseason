// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopSwerve extends Command {
  /** Creates a new TeleopSwerve. */
  SwerveSubsystem m_SwerveSubsystem;
  
  DoubleSupplier m_driveX;
  DoubleSupplier m_driveY;
  DoubleSupplier m_rotation;


  public TeleopSwerve(SwerveSubsystem swerveSub, DoubleSupplier driveX, DoubleSupplier driveY, DoubleSupplier rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveX = driveX;
    m_driveY = driveY;
    m_rotation = rotation;
    m_SwerveSubsystem = swerveSub;

    addRequirements(swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  
  public void execute() {
    // if(Math.abs(m_driveX.getAsDouble()) > 0.2 || Math.abs(m_driveY.getAsDouble()) > 0.2) {
    //   m_SwerveSubsystem.drive(m_driveX.getAsDouble(), m_driveY.getAsDouble(), m_rotation.getAsDouble());
    //   // m_SwerveSubsystem.drive(0, -1 * Math.signum(m_driveY.getAsDouble()), 0);
    // }
    // else if(Math.abs(m_rotation.getAsDouble()) > 0.2) {
    //   m_SwerveSubsystem.drive(0, 0, m_rotation.getAsDouble());
    // }
    // else{
    //   m_SwerveSubsystem.drive(0, 0, 0);
    // }
    double driveX = m_driveX.getAsDouble() > 0.2 ? m_driveX.getAsDouble() : 0;
    double driveY = m_driveY.getAsDouble() > 0.2 ? m_driveY.getAsDouble() : 0;
    double rotation = m_rotation.getAsDouble() > 0.2 ? m_driveX.getAsDouble() : 0;

    m_SwerveSubsystem.drive(driveX, driveY, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
