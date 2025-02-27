// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.DrivebaseModuleConstants;
import frc.robot.utils.SwerveUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private CANSparkMax turnMotor;
  private CANSparkMax driveMotor;

  private PIDController turnPIDController;
  private SparkPIDController drivePIDController;

  private AbsoluteEncoder turnEncoder;
  private RelativeEncoder driveEncoder;

  private int angularOffset;
  private String moduleName;

  private double driveKV;

  public SwerveModule(int turnMotorID, int driveMotorID, int angularOffset, String moduleName, double driveKV) {
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

    turnPIDController = new PIDController(DrivebaseModuleConstants.turnKP, 0, 0);
    drivePIDController = driveMotor.getPIDController();

    turnEncoder = turnMotor.getAbsoluteEncoder();
    driveEncoder = driveMotor.getEncoder();

    this.angularOffset = angularOffset;
    this.moduleName = moduleName;

    this.driveKV = driveKV;

    configureTurnMotor();
    configureDriveMotor();
  }

  private void configureTurnMotor() {
    // Treats 0 and 360 as the same value
    turnPIDController.enableContinuousInput(0, 360);

    turnMotor.setSmartCurrentLimit(35);
    turnMotor.setIdleMode(IdleMode.kCoast);

    turnEncoder.setPositionConversionFactor(DrivebaseModuleConstants.kTurnEncoderPositionFactor);
    turnEncoder.setVelocityConversionFactor(DrivebaseModuleConstants.kTurnEncoderVelocityFactor);

    turnMotor.burnFlash();
  }

  private void configureDriveMotor() {
    drivePIDController.setFeedbackDevice(driveEncoder);

    driveEncoder.setPositionConversionFactor(DrivebaseModuleConstants.kDriveEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(DrivebaseModuleConstants.kDriveEncoderVelocityFactor);

    driveEncoder.setPosition(0);

    drivePIDController.setP(DrivebaseModuleConstants.driveKP);
    drivePIDController.setI(DrivebaseModuleConstants.driveKI);
    drivePIDController.setD(DrivebaseModuleConstants.driveKD);
    drivePIDController.setFF(driveKV);

    driveMotor.setClosedLoopRampRate(8);
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setSmartCurrentLimit(35);

    driveMotor.burnFlash();
  }

  public double getDistance() {
    return driveEncoder.getPosition();
  }

  public double getAngle() {
    return (turnEncoder.getPosition() + angularOffset) % 360;
  }

  public double getTurnPIDOutput(double desiredAngle) {
    return -turnPIDController.calculate(getAngle(), desiredAngle);
  }

  public double getDriveFFOutput(double speed) {
    return MathUtil.clamp(speed * driveKV, -10, 10);
  }

  public void setState(SwerveModuleState state) {
    double[] optimizedModule = SwerveUtil.optimizeModule(getAngle(), state.angle.getDegrees() + 180,
        state.speedMetersPerSecond);

    turnMotor.set(getTurnPIDOutput(optimizedModule[0]));
    // drivePIDController.setReference(optimizedModule[1], ControlType.kVelocity);
    driveMotor.setVoltage(getDriveFFOutput(optimizedModule[1]));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("output current", driveMotor.getOutputCurrent());
    SmartDashboard.putNumber("output voltage", driveMotor.getBusVoltage() * driveMotor.getAppliedOutput());
  }
}
