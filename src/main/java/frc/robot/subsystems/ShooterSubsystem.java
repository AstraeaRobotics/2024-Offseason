// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterStates;


public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax indexerMotor;
  TalonFX bottomShooter;
  TalonFX topShooter;

  TalonFX pivotMotor;

  TalonFXConfiguration configs;

  VelocityVoltage m_bottomVelocity;
  VelocityVoltage m_topVelocity;
  PositionVoltage m_pivotRequest;

  NeutralOut m_brake;

  ShooterStates m_state;
  PIDController m_pid;

  Slot0Configs m_topConfigs;
  Slot1Configs m_bottomConfigs;
  Slot2Configs m_pivotConfigs;

  AbsoluteEncoder pivotEncoder;

  double desiredPosition;
  double manualOffset;

  boolean resetting;
  
  MotorOutputConfigs pivotCurrent;

  public ShooterSubsystem() {
    indexerMotor = new CANSparkMax(11, MotorType.kBrushless);  
    pivotEncoder = indexerMotor.getAbsoluteEncoder(Type.kDutyCycle);

    bottomShooter = new TalonFX(13);
    topShooter = new TalonFX(12);

    pivotMotor = new TalonFX(14);    

    m_brake = new NeutralOut();

    m_topConfigs = new Slot0Configs();
    m_bottomConfigs = new Slot1Configs();
    m_pivotConfigs = new Slot2Configs();

    configureMotors();

    m_topVelocity = new VelocityVoltage(0).withSlot(0);
    m_bottomVelocity = new VelocityVoltage(0).withSlot(1);

    m_pivotRequest = new PositionVoltage(getPivotPosition()).withSlot(2);

    m_state = ShooterStates.kNull;

    desiredPosition = 0;
    manualOffset = 0;
    resetting = false;
  }

  public void configureMotors() {
    indexerMotor.setIdleMode(IdleMode.kCoast);
    indexerMotor.setSmartCurrentLimit(40);
    indexerMotor.setInverted(false);
    
    pivotEncoder.setPositionConversionFactor(48);

    bottomShooter.setNeutralMode(NeutralModeValue.Coast);
    topShooter.setNeutralMode(NeutralModeValue.Coast);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    bottomShooter.setInverted(false);
    topShooter.setInverted(false);
    pivotMotor.setInverted(false);

    m_topConfigs.kS = 0.05;
    m_topConfigs.kV = .12;
    m_topConfigs.kP = 0.2;
    m_topConfigs.kI = 0;
    m_topConfigs.kD = 0;

    m_bottomConfigs.kS = 0.05;
    m_bottomConfigs.kV = .12;
    m_bottomConfigs.kP = 0.2;
    m_bottomConfigs.kI = 0;
    m_bottomConfigs.kD = 0;

    m_pivotConfigs.kS = 0.1;
    m_pivotConfigs.kV = .2;
    m_pivotConfigs.kP = .35; // change to 0.5 and test
    m_pivotConfigs.kI = 0.02;
    m_pivotConfigs.kD = 0;

    bottomShooter.getConfigurator().apply(m_bottomConfigs);
    topShooter.getConfigurator().apply(m_topConfigs);

    pivotMotor.getConfigurator().apply(m_pivotConfigs);

    pivotMotor.setPosition(0);
    desiredPosition = .75;
  }

  public void prepIntake() {
    bottomShooter.setInverted(true);
    topShooter.setInverted(true);
  }


  public void prepShoot() {
    bottomShooter.setInverted(false);
    topShooter.setInverted(false);
  }

  public void prepAmp(){

    topShooter.setInverted(true);
  }

  public void prepPostIntake() {
    topShooter.setInverted(false);
  }

  public void spinIndexer(double speed){
    indexerMotor.set(speed);
  }

  public void spinBottomShooter(double speed){
    // bottomShooter.setControl(m_bottomVelocity.withVelocity(speed).withFeedForward(ff));
    bottomShooter.setControl(m_bottomVelocity.withVelocity(speed));
  }

  public void spinTopShooter(double speed){
    // topShooter.setControl(m_topVelocity.withVelocity(speed).withFeedForward(ff));
    topShooter.setControl(m_topVelocity.withVelocity(speed));
  }

  public void stopShooter() {
    topShooter.setControl(m_brake);
    bottomShooter.setControl(m_brake);
  }

  public double getShooterVelocity() {
    return topShooter.getVelocity().getValueAsDouble();
  }

  public double getShooterError() {
    return topShooter.getClosedLoopError().getValueAsDouble();
  }

  public void spinPivotMotor(double position) {
    pivotMotor.setControl(m_pivotRequest.withPosition(position));
  }

  public void setPivotVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  public void stopPivot() {
    pivotMotor.setControl(m_brake);
  }

  public void setPivotCoast() {
    pivotMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setPivotBrake() {
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getPivotPosition() {
    return pivotMotor.getPosition().getValueAsDouble();
  }

  public double getPivotError() {
    return pivotMotor.getClosedLoopError().getValueAsDouble();
  }

  public void setState(ShooterStates state) {
    m_state = state;
  }

  public ShooterStates getState() {
    return m_state;
  }

  public void incrementPivot(double increment) {
    if (getPivotPosition() < 70) manualOffset += increment;

    if (getPivotPosition() > 70) manualOffset = 70;
  }

  public void decrementPivot(double decrement) {
    if (getPivotPosition() >= decrement) manualOffset -= decrement;

    if (getPivotPosition() < 0) manualOffset = 0;
  }

  public void setManualOffset(double newPosition) {
    manualOffset = newPosition;
  }

  public void setDesiredPosition(double newPosition) {
    desiredPosition = newPosition;
  }

  public void setPivotPosition (double newPosition) {
    pivotMotor.setPosition(newPosition);
  }

  public void setReset(boolean isReset) {
    resetting = isReset;
  }

  public void checkDesiredPosition() {

    // desiredPosition = m_state.getPivotSetpoint();

    if (desiredPosition < ShooterConstants.kPivotGroundPosition) desiredPosition = ShooterConstants.kPivotGroundPosition;
    if (desiredPosition > 52) desiredPosition = 52;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Relative Encoder Position: ", getPivotPosition());
    SmartDashboard.putNumber("Pivot Absolute Encoder Position: ", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Error", getPivotError());
    SmartDashboard.putNumber("Desired Position: ", desiredPosition);
    SmartDashboard.putBoolean("Resetting", resetting);
    SmartDashboard.putNumber("shooter velocity", getShooterVelocity());
    checkDesiredPosition();

    if (!resetting || getState() != ShooterStates.kNull) spinPivotMotor(desiredPosition + manualOffset);
    if (getState() == ShooterStates.kNull) {
      setPivotBrake();
      stopPivot();
    }
    if (resetting) stopPivot();
  }
}
