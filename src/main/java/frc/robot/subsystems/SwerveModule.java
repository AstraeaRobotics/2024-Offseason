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
import frc.robot.Constants.DrivebaseModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private CANSparkMax turnMotor;
  private CANSparkMax driveMotor;

  private SparkPIDController turnPIDController;
  private SparkPIDController drivePIDController;

  private AbsoluteEncoder turnEncoder;
  private RelativeEncoder driveEncoder;

  private int angularOffset;


  public SwerveModule(int turnMotorID, int driveMotorID, int angularOffset) {
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

    turnPIDController = turnMotor.getPIDController();
    drivePIDController = driveMotor.getPIDController();

    turnEncoder = turnMotor.getAbsoluteEncoder();
    driveEncoder = driveMotor.getEncoder();

    this.angularOffset = angularOffset;

    configureTurnMotor();
    configureDriveMotor();
  }

  private void configureTurnMotor() {
    turnMotor.setInverted(false);
    turnPIDController.setFeedbackDevice(turnEncoder);

    turnMotor.setSmartCurrentLimit(35);
    turnMotor.setIdleMode(IdleMode.kBrake);

    turnEncoder.setPositionConversionFactor(DrivebaseModuleConstants.kTurnEncoderPositionFactor);
    turnEncoder.setVelocityConversionFactor(DrivebaseModuleConstants.kTurnEncoderVelocityFactor);

    // Turns in opposite direction if abs value of error is > 90 degrees
    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(0);
    turnPIDController.setPositionPIDWrappingMaxInput(90);

    turnPIDController.setP(DrivebaseModuleConstants.turnKP);
    turnPIDController.setI(DrivebaseModuleConstants.turnKI);
    turnPIDController.setD(DrivebaseModuleConstants.turnKD);

    turnMotor.burnFlash();

    // Todo: need to set ff
    // Todo: need to set turn encoder position
  }

  private void configureDriveMotor() {
    driveMotor.setInverted(false);
    drivePIDController.setFeedbackDevice(driveEncoder);

    driveEncoder.setPositionConversionFactor(DrivebaseModuleConstants.kDriveEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(DrivebaseModuleConstants.kDriveEncoderVelocityFactor);

    driveEncoder.setPosition(0);

    driveMotor.burnFlash();
  }

  public double getDistance(){
    return driveEncoder.getPosition();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(turnEncoder.getPosition());
  }

  public void setState(SwerveModuleState state) {
    turnPIDController.setReference(state.angle.getDegrees(), ControlType.kPosition);
    drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
