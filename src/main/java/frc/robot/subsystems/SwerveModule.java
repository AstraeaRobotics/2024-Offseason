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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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


  public SwerveModule(int turnMotorID, int driveMotorID, int angularOffset, String moduleName) {
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

    turnPIDController = new PIDController(DrivebaseModuleConstants.turnKP, 0, 0);
    drivePIDController = driveMotor.getPIDController();

    turnEncoder = turnMotor.getAbsoluteEncoder();
    driveEncoder = driveMotor.getEncoder();

    this.angularOffset = angularOffset;
    this.moduleName = moduleName;


    configureMotors();
  }

  private void configureMotors() {
    // Turn motor configuration
    turnPIDController.enableContinuousInput(0, 360);

    turnEncoder.setPositionConversionFactor(DrivebaseModuleConstants.kTurnEncoderPositionFactor);
    turnEncoder.setVelocityConversionFactor(DrivebaseModuleConstants.kTurnEncoderVelocityFactor);

    turnMotor.setSmartCurrentLimit(35);
    turnMotor.setClosedLoopRampRate(8);
    turnMotor.setIdleMode(IdleMode.kCoast);

    turnMotor.burnFlash();

    // Drive motor configuration
    drivePIDController.setFeedbackDevice(driveEncoder);

    driveEncoder.setPositionConversionFactor(DrivebaseModuleConstants.kDriveEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(DrivebaseModuleConstants.kDriveEncoderVelocityFactor);
    driveEncoder.setPosition(0);

    drivePIDController.setP(DrivebaseModuleConstants.driveKP);
    drivePIDController.setI(DrivebaseModuleConstants.driveKI);
    drivePIDController.setD(DrivebaseModuleConstants.driveKD);
    drivePIDController.setFF(DrivebaseModuleConstants.driveKV);

    driveMotor.setClosedLoopRampRate(8);
    driveMotor.setSmartCurrentLimit(35);
    driveMotor.setIdleMode(IdleMode.kBrake);

    driveMotor.burnFlash();
  }

  public double getDistance(){
    return driveEncoder.getPosition();
  }

  public double getAngle() {
    return (turnEncoder.getPosition() + angularOffset) % 360;
  }

  public SwerveModulePosition getModulePosition() {
    double position = getDistance();
    Rotation2d angle = Rotation2d.fromDegrees(-getAngle());

    return new SwerveModulePosition(position, angle);
  }

  public void setState(SwerveModuleState state) {
    double[] optimizedModule = SwerveUtil.optimizeModule(getAngle(), state.angle.getDegrees() + 180, state.speedMetersPerSecond);

    turnMotor.set(-turnPIDController.calculate(getAngle(), optimizedModule[0]));
    drivePIDController.setReference(optimizedModule[1], ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("output current", driveMotor.getOutputCurrent());
    // SmartDashboard.putNumber("output voltage", driveMotor.getBusVoltage() * driveMotor.getAppliedOutput());
  }
}
