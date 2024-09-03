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

  // private SparkPIDController turnPIDController;
  PIDController turnPIDController;
  private SparkPIDController drivePIDController;

  private AbsoluteEncoder turnEncoder;
  private RelativeEncoder driveEncoder;

  private int angularOffset;
  private String moduleName;

  private boolean isInverted;


  public SwerveModule(int turnMotorID, int driveMotorID, int angularOffset, String moduleName, boolean isInverted) {
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

    turnPIDController = new PIDController(DrivebaseModuleConstants.turnKP, 0, 0);
    drivePIDController = driveMotor.getPIDController();

    turnEncoder = turnMotor.getAbsoluteEncoder();
    driveEncoder = driveMotor.getEncoder();

    this.angularOffset = angularOffset;
    this.moduleName = moduleName;

    this.isInverted = isInverted;


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
    driveMotor.setInverted(false);
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

  public void setState(SwerveModuleState state, boolean isRotating) {
    // double rawAngle = (state.angle.getDegrees() + 360) % 360;
    double goalAngle = state.angle.getDegrees() * -1;
    double currentAngle = getAngle() - 180;
    double speed = 0;


    // Turn PID
    double error = SwerveUtil.remapAngle(currentAngle, goalAngle);
    double PIDOutput = error * DrivebaseModuleConstants.turnKP;

    speed = SwerveUtil.remapSpeed(goalAngle - currentAngle, state.speedMetersPerSecond);

    if(isInverted && isRotating) {
      speed = -speed;
    }


    turnMotor.set(PIDOutput);
    driveMotor.set(speed * 0.25);

    SmartDashboard.putNumber("ogD", goalAngle - currentAngle);
    // SmartDashboard.putNumber("goal angle", goalAngle);
    // SmartDashboard.putNumber("current angle", currentAngle);
    SmartDashboard.putNumber("speed", speed * 0.15);
    SmartDashboard.putBoolean("isRotating", isRotating);
    // SmartDashboard.putNumber("pid output", PIDOutput);

    

    // speed numbers
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
