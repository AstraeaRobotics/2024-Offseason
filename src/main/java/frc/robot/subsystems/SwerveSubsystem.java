// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;
  AHRS gyro;

  SwerveModule[] swerveModules;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;
  SwerveModule frontLeftModule;
  SwerveModule frontRightModule;

  Translation2d m_frontLeftLocation = new Translation2d(DrivebaseConstants.kWheelBase / 2, -DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_frontRightLocation = new Translation2d(DrivebaseConstants.kWheelBase / 2, DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_backLeftLocation = new Translation2d(-DrivebaseConstants.kWheelBase / 2, -DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_backRightLocation = new Translation2d(-DrivebaseConstants.kWheelBase / 2, DrivebaseConstants.kTrackWidth / 2); 

  public SwerveModuleState backLeftModuleState;
  public SwerveModuleState backRightModuleState;
  public SwerveModuleState frontLeftModuleState;
  public SwerveModuleState frontRightModuleState;

  public SwerveSubsystem() {
    kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    gyro = new AHRS();

    swerveModules = new SwerveModule[4];

    swerveModules[0] = new SwerveModule(2, 1, 180, "front left", false); // Front left
    swerveModules[1] = new SwerveModule(4, 3, 270, "front right", true); // Front right
    swerveModules[2] = new SwerveModule(6, 5, 90, "back left", true); // Back left   
    swerveModules[3] = new SwerveModule(8, 7, 0, "back right", false); // Back right

    gyro.reset();
  }

  public void drive(double driveX, double driveY, double rotation, boolean isRotating) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveY, driveX, rotation, Rotation2d.fromDegrees(getHeading()));
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
    // SmartDashboard.putNumber("driveX", driveX);
    // SmartDashboard.putNumber("drive y", driveY);

    swerveModules[0].setState(swerveModuleStates[0], isRotating); // Front left
    swerveModules[1].setState(swerveModuleStates[1], isRotating); // Front right
    swerveModules[2].setState(swerveModuleStates[2], isRotating); // Back left
    swerveModules[3].setState(swerveModuleStates[3], isRotating); // Back right

  }

  public double getHeading() {
    return (gyro.getAngle() + 90) % 360;
  }

  public void resetGyro() {
    gyro.reset();
  }

  // ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(frontRightModuleState);

  // double forward = chassisSpeeds.vxMetersPerSecond;
  // double sideways = chassisSpeeds.vyMetersPerSecond;
  // double angular = chassisSpeeds.omegaRadiansPerSecond;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("current heading", getHeading());

  }
}
