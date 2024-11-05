// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.proto.Photon;

import com.kauailabs.navx.frc.AHRS;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;
  AHRS gyro;

  SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  SwerveDriveOdometry swerveDriveOdometry;

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

  PhotonCamera camera;

  Pose2d odomPose;
  Pose2d visionPose;

  StructPublisher<Pose2d> publisher;
  StructPublisher<Pose2d> arrayPublisher;

  public SwerveSubsystem() {
    kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    gyro = new AHRS();

    swerveModules = new SwerveModule[4];
    swerveModules[0] = new SwerveModule(2, 1, 270, "front left"); // Front left
    swerveModules[1] = new SwerveModule(4, 3, 0, "front right"); // Front right
    swerveModules[2] = new SwerveModule(6, 5, 180, "back left"); // Back left   
    swerveModules[3] = new SwerveModule(8, 7, 90, "back right"); // Back right

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions(), new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
    swerveDriveOdometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    camera = new PhotonCamera("limelight");

    odomPose = new Pose2d();
    visionPose = new Pose2d();
    publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
    arrayPublisher = NetworkTableInstance.getDefault().getStructTopic("MyPoseArray", Pose2d.struct).publish();

    

    gyro.reset();
  }

  public void drive(double driveX, double driveY, double rotation) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-driveY, driveX, rotation, Rotation2d.fromDegrees(getHeading()));
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

    swerveModules[0].setState(swerveModuleStates[0]); // Front left
    swerveModules[1].setState(swerveModuleStates[1]); // Front right
    swerveModules[2].setState(swerveModuleStates[2]); // Back left
    swerveModules[3].setState(swerveModuleStates[3]); // Back right
  }

  public double getHeading() {
    return gyro.getYaw();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = swerveModules[0].getModulePosition();
    positions[1] = swerveModules[1].getModulePosition();
    positions[2] = swerveModules[2].getModulePosition();
    positions[3] = swerveModules[3].getModulePosition();
    
    return positions;
  }

  public Pose2d getRobotPose2d() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrivePoseEstimator.update(Rotation2d.fromDegrees(-getHeading()), getModulePositions());
    odomPose = getRobotPose2d();
    publisher.set(visionPose);
  }
}
