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
    public static final double turnKP = 0;
    public static final double turnKI = 0;
    public static final double turnKD = 0;

    // FeedForward Constants

  }
}
