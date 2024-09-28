// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivebaseModuleConstants {
    // Physical Constants
    public static final double kDriveGearRatio = 3.56;
    public static final double kWheelDiameter = Units.inchesToMeters(3);

    // Conversion Factors
    public static final int kTurnEncoderPositionFactor = 360;
    public static final int kTurnEncoderVelocityFactor = 60; // not sure about this

    public static final double kDriveEncoderPositionFactor = (1 / kDriveGearRatio) * 2 * Math.PI * (kWheelDiameter / 2);
    public static final double kDriveEncoderVelocityFactor = 1/(60 * kDriveGearRatio);

    // PID Constants (change later)
    public static final double turnKP = 0.004;
    public static final double turnKI = 0;
    public static final double turnKD = 0;

    public static final double driveKP = 0.01;
    public static final double driveKI = 0;
    public static final double driveKD = 0;

    // FeedForward Constants
    public static final double turnKV = 0.00005; // 0.31
    public static final double driveKV = 0.65; // 0.45

  }

  public static class DrivebaseConstants {
    // Physical constants
    public static final double kWheelBase = Units.inchesToMeters(20);/* the distance between the front and rear wheels */
    public static final double kTrackWidth = Units.inchesToMeters(22); /* the distance between left and right wheels */
  }

  public static class ShooterConstants {

    public static final double kShooterVelocity = 0; // TODO: determine shooter velocity

    public static final double kShooterSpeed = 80;
    public static final double kPoopSpeed = 35;
    public static final double kTrapSpeed = 40;


    public static final double kPivotMotorGearRatio = 260;
    public static final double kPivotMotorConversionFactor = (360) / kPivotMotorGearRatio;
    public static final double kPivotMinPosition = 0;
    public static final double kPivotMaxPosition = 45;
    
    public static final double kPivotSpeed = 0.05;
    public static final double kPivotTolerance = 0.1;

    public static final double kPivotGroundPosition = 1.625;

    public enum ShooterStates {
      kGround(kPivotGroundPosition, -1, -1),
      kSource(44.8, 0, 1.08585),
      kSpeaker(30, 1, 2.032), // originally 31.5
      kFeed(17, 1, 1),
      kSpeakerSide(32.07, 1, 2.032),
      kSpeaker2(24.2, 1, 2.032), // 26.2
      kAmp(44.88,  2, 0.889),
      kTrap(0, 3, 1.4351),
      kNull(0, 5, 0);


      private double pivotSetpoint;
      private int pipelineID;
      private double targetHeight;

      private ShooterStates(double pivotSetpoint, int pipelineID, double targetHeight) {
        this.pivotSetpoint = pivotSetpoint;
        this.pipelineID = pipelineID;
        this.targetHeight = targetHeight;
      }

      public double getPivotSetpoint() {
        return pivotSetpoint;
      }

      public int getPipelineID() {
        return pipelineID;
      }

     public double getTargetHeight() {
        return targetHeight;
      }
    }
  }

  public static class IntakeConstants {
    public static final double intakeKS = 0.24;
    public static final double intakeKV = 0.133;

    public static final double preIntakeKS = .13;
    public static final double preIntakeKV = .1241;
  }
}
