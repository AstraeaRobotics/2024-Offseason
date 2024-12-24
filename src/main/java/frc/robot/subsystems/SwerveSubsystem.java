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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.utils.LimelightUtil;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.utils.LimelightHelpers;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  SwerveDriveKinematics kinematics;
  AHRS gyro;

  SwerveModule[] swerveModules;

  Translation2d m_frontLeftLocation = new Translation2d(DrivebaseConstants.kWheelBase / 2, -DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_frontRightLocation = new Translation2d(DrivebaseConstants.kWheelBase / 2, DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_backLeftLocation = new Translation2d(-DrivebaseConstants.kWheelBase / 2, -DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_backRightLocation = new Translation2d(-DrivebaseConstants.kWheelBase / 2, DrivebaseConstants.kTrackWidth / 2); 

  SwerveDrivePoseEstimator swerveDrivePoseEstimator;

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
    publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetRobotPose, 
      this::getRobotRelativeSpeeds, 
      this::driveRobotRelative, 
      new HolonomicPathFollowerConfig(3.6, 0.57, new ReplanningConfig()), 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;}, 
      this);
    
    gyro.reset();
  }

  public void drive(double driveX, double driveY, double rotation) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-driveY, driveX, rotation, Rotation2d.fromDegrees(getHeading()));
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
    
    setModuleStates(swerveModuleStates);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

    setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    // for(int i = 0; i < moduleStates.length; i++) {
    //   swerveModules[i].setState(moduleStates[i]);
    // }
    swerveModules[0].setState(moduleStates[0]);
    swerveModules[1].setState(moduleStates[1]);
    // swerveModules[2].setState(moduleStates[2]);
    swerveModules[3].setState(moduleStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for(int i = 0; i < 4; i++) {
      moduleStates[i] = swerveModules[i].getModuleState();
    }
    return moduleStates;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double getHeading() {
    return gyro.getYaw();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for(int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getModulePosition();
    }
    
    return positions;
  }

  public Pose2d getPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public void resetRobotPose(Pose2d pose) {
    swerveDrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(-getHeading()), getModulePositions(), pose);
  }

  // public void updatePose() {
  //   double[] m_limelightPose = LimelightUtil.getBotPose();

  //   if (m_limelightPose != null && m_limelightPose.length >= 3 /* idk what num this shd be still */) {
  //     Pose2d m_limelightPose2d = new Pose2d(
  //       m_limelightPose[0], // x
  //       m_limelightPose[1], // y
  //       Rotation2d.fromDegrees(m_limelightPose[2]) // rot
  //     );

  //     swerveDrivePoseEstimator.addVisionMeasurement(m_limelightPose2d, Timer.getFPGATimestamp());

  //     // copied above from here: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html
  //   }
  // }

  public void updatePoseNEW() {
    LimelightHelpers.SetRobotOrientation("limelight", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0); // idk what these zeros are

    LimelightHelpers.PoseEstimate apriltag = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    
    double[] m_limelightPose = LimelightUtil.getBotPose();

    Pose2d m_limelightPose2d = null; // only for the conditional at the end

    if (m_limelightPose != null && m_limelightPose.length >= 3 /* idk what num this shd be still */) {
        m_limelightPose2d = new Pose2d(
            m_limelightPose[0], // x
            m_limelightPose[1], // y
            Rotation2d.fromDegrees(m_limelightPose[2]) // rot
        );
    }

    boolean noUpdate = false;

    if (gyro.getRate() > 720 /*where is this number from? */ || apriltag.tagCount == 0) {
        noUpdate = true;
    }

    if (noUpdate == false) {
        if (m_limelightPose2d != null) {
            swerveDrivePoseEstimator.addVisionMeasurement(m_limelightPose2d, Timer.getFPGATimestamp()); // both
        }
        swerveDrivePoseEstimator.addVisionMeasurement(apriltag.pose, Timer.getFPGATimestamp()); // if no limelight, only apriltag
    }
}

  
  @Override
  public void periodic() {
    // updatePose();
    updatePoseNEW();
    swerveDrivePoseEstimator.update(Rotation2d.fromDegrees(-getHeading()), getModulePositions());
    publisher.set(getPose());
  }
}
