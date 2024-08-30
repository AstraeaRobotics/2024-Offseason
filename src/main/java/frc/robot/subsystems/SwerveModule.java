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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private CANSparkMax turnMotor;
  private CANSparkMax driveMotor;

  // private SparkPIDController turnPIDController;
  PIDController turnPIDController;
  private SparkPIDController drivePIDController;

  private AbsoluteEncoder turnEncoder;
  private RelativeEncoder driveEncoder;

  private int angularOffset;
  private String moduleName;

  public SwerveModule(int turnMotorID, int driveMotorID, int angularOffset, String moduleName) {
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

    turnPIDController = new PIDController(DrivebaseModuleConstants.turnKP, 0, 0);
    drivePIDController = driveMotor.getPIDController();

    turnEncoder = turnMotor.getAbsoluteEncoder();
    driveEncoder = driveMotor.getEncoder();

    this.angularOffset = angularOffset;
    this.moduleName = moduleName;

    configureTurnMotor();
    configureDriveMotor();
  }

  private void configureTurnMotor() {
    turnMotor.setInverted(false);
    // turnPIDController.setFeedbackDevice(turnEncoder);

    turnMotor.setSmartCurrentLimit(35);
    turnMotor.setIdleMode(IdleMode.kCoast);

    turnEncoder.setPositionConversionFactor(DrivebaseModuleConstants.kTurnEncoderPositionFactor);
    turnEncoder.setVelocityConversionFactor(DrivebaseModuleConstants.kTurnEncoderVelocityFactor);

    turnMotor.burnFlash();

    // Todo: need to set turn encoder position
  }

  private void configureDriveMotor() {
    driveMotor.setInverted(true);
    drivePIDController.setFeedbackDevice(driveEncoder);

    driveMotor.setSmartCurrentLimit(35);

    driveEncoder.setPositionConversionFactor(DrivebaseModuleConstants.kDriveEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(DrivebaseModuleConstants.kDriveEncoderVelocityFactor);

    driveEncoder.setPosition(0);

    drivePIDController.setP(DrivebaseModuleConstants.driveKP);
    drivePIDController.setI(DrivebaseModuleConstants.driveKI);
    drivePIDController.setD(DrivebaseModuleConstants.driveKD);
    drivePIDController.setFF(DrivebaseModuleConstants.driveKV);

    driveMotor.burnFlash();
  }

  public double getDistance(){
    return driveEncoder.getPosition();
  }

  public double getAngle() {
    return (turnEncoder.getPosition() + angularOffset) % 360;
  }

  public void setState(SwerveModuleState state) {
    double desiredAngle = state.angle.getDegrees();
    double currentAngle = getAngle() - 180;
    // double goalAngle = SwerveUtil.remapAngle(controllerAngle, rawAngle);
    double error = SwerveUtil.remapAngle(currentAngle, desiredAngle);
    double PIDOutput = error * DrivebaseModuleConstants.turnKP;

    // double PIDOutput = turnPIDController.calculate(goalAngle, controllerAngle);
    // double speed = SwerveUtil.remapSpeed(desiredAngle, state.speedMetersPerSecond);

    // turnMotor.set(PIDOutput);
    // drivePIDController.setReference(speed, ControlType.kVelocity);

    SmartDashboard.putNumber("raw desired angle", desiredAngle);
    SmartDashboard.putNumber("current angle", currentAngle);
    // SmartDashboard.putNumber("error", error);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
