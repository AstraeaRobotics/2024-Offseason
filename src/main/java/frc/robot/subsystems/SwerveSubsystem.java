// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  SwerveDriveKinematics kinematics;

  Translation2d m_frontLeftLocation = new Translation2d(DrivebaseConstants.kWheelBase / 2, -DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_frontRightLocation = new Translation2d(DrivebaseConstants.kWheelBase / 2, DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_backLeftLocation = new Translation2d(-DrivebaseConstants.kWheelBase / 2, -DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_backRightLocation = new Translation2d(-DrivebaseConstants.kWheelBase / 2, DrivebaseConstants.kTrackWidth / 2); 
  

  public SwerveSubsystem() {
    kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
